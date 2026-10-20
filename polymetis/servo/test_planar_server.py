"""Exercise planar parameter updates against synthetic feedback, never FCI."""
import argparse
import io
import json
import socket
import subprocess
import threading
import time
from pathlib import Path

import grpc
import numpy as np
import torch
import polymetis
import polymetis_pb2 as pb
import polymetis_pb2_grpc as rpc
from polymetis.robot_interface import BaseRobotInterface, ParamDictContainer
from torchcontrol.models import RobotModelPinocchio
from torchcontrol.policies.default_controller import DefaultController
from torchcontrol.policies.planar_impedance import PlanarImpedanceControl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--server', required=True)
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--config', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    torch.set_num_threads(1)
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0)); port = s.getsockname()[1]
    log = open(args.output + '.log', 'wb')
    process = subprocess.Popen(['taskset', '-c', '3', args.server, '-r', '-s', '127.0.0.1', '-p', str(port)], stdout=log, stderr=subprocess.STDOUT)
    channel = grpc.insecure_channel('127.0.0.1:' + str(port))
    stop = threading.Event(); rows = []; updates = []; errors = []; worker = None
    phase = ['startup']
    try:
        grpc.channel_ready_future(channel).result(timeout=5)
        stub = rpc.PolymetisControllerServerStub(channel)
        default = torch.jit.script(DefaultController([40.] * 7, [8.] * 7))
        data = io.BytesIO(); torch.jit.save(default, data)
        config = json.loads(Path(args.config).read_text())
        meta = pb.RobotClientMetadata(hz=1000, dof=7, ee_link_name=config['ee_link_name'], urdf_file=Path(args.urdf).read_text(),
            polymetis_version=polymetis.__version__, default_controller=data.getvalue(), default_Kq=[40.]*7, default_Kqd=[8.]*7)
        stub.InitRobotClient(meta, timeout=3)
        q = torch.tensor([0.06274,0.23460,-0.14586,-2.01374,0.03023,2.36643,1.07208])
        def hardware():
            due = time.monotonic()
            while not stop.is_set():
                state = pb.RobotState(joint_positions=q.tolist(), joint_velocities=[0.]*7,
                    motor_torques_measured=[0.]*7,motor_torques_external=[0.]*7)
                state.timestamp.GetCurrentTime(); start = time.perf_counter_ns()
                try:
                    value = stub.ControlUpdate(state,timeout=.5)
                    assert len(value.joint_torques) == 7 and np.isfinite(value.joint_torques).all()
                    rows.append([phase[0],(time.perf_counter_ns()-start)/1e6])
                except Exception as exc:
                    errors.append(str(exc));stop.set();break
                due += .001; stop.wait(max(0,due-time.monotonic()))
        worker = threading.Thread(target=hardware);worker.start();time.sleep(.1)
        model = RobotModelPinocchio(args.urdf,config['ee_link_name']); position,_=model.forward_kinematics(q)
        extent=torch.tensor(config['xy_half_extent'])
        fields=('stiffness','damping','wrench_limits','max_speed','max_acceleration','velocity_time_constant',
            'max_tracking_error','normal_travel','preload','preload_ramp_s','joint_damping','timeout')
        optional=('start_in_contact','normal_bounds','normal_speed','normal_acceleration',
            'free_normal_stiffness','free_normal_damping','free_normal_force_up')
        policy=PlanarImpedanceControl(q,model,torch.eye(3),position[:2]-extent,position[:2]+extent,
            **{k:config[k] for k in fields},**{k:config[k] for k in optional if k in config})
        stub.SetController(BaseRobotInterface._get_msg_generator(torch.jit.script(policy))(),timeout=5)
        phase[0]='paused';time.sleep(.2)
        for i in range(1,101):
            phase[0]='first_update' if i == 1 else 'repeated_updates'
            params=ParamDictContainer({'velocity_desired':torch.tensor([.02 if i%20<10 else -.02,0.]),
                'normal_velocity_desired':torch.tensor([.02 if i<50 else 0.]),
                'command_contact':torch.tensor([float(i>=50)]),
                'command_sequence':torch.tensor([float(i)],dtype=torch.float64),'command_paused':torch.tensor([float(i%20==0)])})
            msg=BaseRobotInterface._get_msg_generator(torch.jit.script(params))
            start=time.perf_counter_ns()
            stub.UpdateController(msg(),timeout=2)
            updates.append((time.perf_counter_ns()-start)/1e6);time.sleep(.01)
        phase[0]='stale';time.sleep(.25)
        assert stub.GetEpisodeInterval(pb.Empty(),timeout=.2).end == -1
        result={'synthetic_feedback_only':True,'errors':errors,'samples':len(rows),'phase_rpc_ms':{},
            'first_update_ms':updates[0],'update_p99_ms':float(np.percentile(updates[1:],99))}
        for name in set(x[0] for x in rows):
            vals=[x[1] for x in rows if x[0]==name]
            result['phase_rpc_ms'][name]={'max':max(vals),'p99':float(np.percentile(vals,99)),'over_10ms':sum(x>10 for x in vals)}
        Path(args.output).write_text(json.dumps(result,indent=2));print(json.dumps(result))
        assert not errors, errors
        assert len(rows) > 1000
        for name in ('first_update', 'repeated_updates', 'stale'):
            assert result['phase_rpc_ms'][name]['max'] < 10, result
    finally:
        stop.set()
        if worker:worker.join(2)
        channel.close();process.terminate()
        try:process.wait(3)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close()

if __name__ == '__main__':main()

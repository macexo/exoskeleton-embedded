"""Host regression for the actual motor service, with mocked HAL/CAN transport.
Run with Python and gcc on PATH; no hardware is driven.
"""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[2] / "Core/Src/main.c").read_text()
start = source.index("static float requested_current")
controller = source[start:source.index("/**", start)]
STUB = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#define JOINT_MOTOR_ID 107
#define NODE_ID 4
#define CURRENT_LIMIT_A 5.0f
#define TORQUE_PER_AMP (0.159f * 9.0f)
#define CURRENT_REFRESH_MS 50
#define MOTOR_TELEMETRY_MS 10
#define CAN_MSG_CURRENT_SETPOINT 6
#define COMMAND_TIMEOUT_MS 200
#define CAN_ID_GLOBAL_RESET 0x67
#define CAN_MSG_ESTOP 0
#define CAN_MSG_MOTOR_STATUS 2
#define CAN_NODE_PI 0
#define CAN_BUILD_ID(t,s,d) ((t<<7)|(s<<4)|d)
#define HAL_CAN_AbortTxRequest(...) ((void)0)
#define __HAL_CAN_GET_FLAG(...) 0
#define __HAL_CAN_CLEAR_FLAG(...) ((void)0)
#define HAL_CAN_GetTxMailboxesFreeLevel(...) 3
#define NVIC_SystemReset() ((void)0)
typedef struct {unsigned id,dlc,is_extended;uint8_t data[8];} CanFrame;
static CanFrame input; static int pending, writes, reports, origins; static float amps;
static int setpoints, accept=1; static int32_t setpoint;
static unsigned tick; static uint8_t payload[8];
unsigned HAL_GetTick(void){return tick;}
int can_recv(CanFrame *f){if(!pending)return 0; *f=input; pending=0;return 1;}
unsigned can_get_msg_type(unsigned id){return id>>7;}
int can_parse_torque_cmd(CanFrame *f,float *v){int16_t r; memcpy(&r,f->data,2); *v=r/1000.f;return f->id==0x84;}
int comm_can_set_current(unsigned id,float a){assert(id==107);amps=a; writes++;return 1;}
int comm_can_set_origin(unsigned id,int mode){assert(id==107 && !mode); origins++;return 1;}
int can_send_std(unsigned id,uint8_t *data,unsigned len){if(!accept)return 0;
 if(id==0x340){assert(len==4);memcpy(&setpoint,data,4);setpoints++;return 1;}
 assert(id==0x140 && len==8);memcpy(payload,data,8);reports++;return 1;}

unsigned ak70_9_feedback_id(unsigned id){return 0x2900|id;}
'''
TEST = r'''
int main(void){
 motor_service();assert(setpoints==1 && setpoint==0);
 tick=9;motor_service();assert(setpoints==1);
 tick=10;motor_service();assert(setpoints==2);
 input=(CanFrame){.id=0x84,.dlc=2,.data={0xff,0x7f}};pending=1;
 tick=20;motor_service();assert(amps==5 && setpoint==5000);
 input.data[0]=0;input.data[1]=0x80;pending=1;
 tick=30;motor_service();assert(amps==-5 && setpoint==-5000);
 input.data[0]=0x58;input.data[1]=0x1b;pending=1; // 7 Nm
 tick=40;motor_service();assert(setpoint==(int32_t)(7.0f/TORQUE_PER_AMP*1000));
 tick=239;motor_service();assert(amps>0);
 tick=240;motor_service();assert(amps==0 && setpoint==0); // immediate timeout zero
 input=(CanFrame){.id=0x296b,.dlc=8,.is_extended=1,.data={0xff,0xfb,0,2,0xff,0x9c,40,1}};
 pending=1;motor_service();uint8_t expected[]={0xfb,0xff,2,0,0x9c,0xff,40,1};
 assert(reports==1 && !memcmp(payload,expected,8));
 tick=249;pending=1;motor_service();assert(reports==1);
 tick=250;pending=1;motor_service();assert(reports==2);
 tick=251;input.data[7]=2;pending=1;motor_service();assert(reports==3);
 tick=260;accept=0;int count=setpoints;motor_service();assert(setpoints==count);
 accept=1;motor_service();assert(setpoints==count+1); // mailbox failure retries
 input=(CanFrame){.id=0x84,.dlc=2,.data={0xff,0x7f}};pending=1;tick=270;motor_service();assert(setpoint==5000);
 input=(CanFrame){.id=0,.dlc=1};pending=1;tick=271;motor_service();assert(amps==0 && setpoint==0 && estop_latched);
 input=(CanFrame){.id=0x84,.dlc=2,.data={0xff,0x7f}};pending=1;tick=281;motor_service();assert(amps==0 && setpoint==0);
 reset_pending=1;count=setpoints;tick=300;motor_service();assert(setpoints==count);
}
'''
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory)
    (path / "test.c").write_text(STUB + controller + TEST)
    exe = path / "test.exe"
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Werror", str(path / "test.c"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: 100 Hz timing, status payload, clamp/conversion, zero timeout/e-stop, mailbox retry, reset")

`timescale 1ns/1ps
// Experimental local FLU dead reckoning. Never use as an unaided position lock.
// Native FLU acceleration (16384 counts/g); controller roll/pitch and CW yaw rate.
module PositionEstimator #(
 parameter integer CLK_FREQ=10_000_000, SAMPLE_HZ=500, CALIBRATION_SAMPLES=64,
 parameter integer MAX_TRACK_SECONDS=10, TRACK_Z=1
)(
 input wire clk,rst,enable,sample_valid,imu_valid,accel_clipped,
 input wire signed [15:0] accel_x,accel_y,accel_z,roll_cd,pitch_cd,yaw_rate_cdps,
 output reg calibrated,degraded,
 output wire estimate_valid,
 output reg signed [31:0] x_mm,y_mm,z_mm,vx_mmps,vy_mmps,vz_mmps
);
 localparam integer PERIOD=CLK_FREQ/SAMPLE_HZ;
 localparam integer DT_Q16=(65536+SAMPLE_HZ/2)/SAMPLE_HZ;
 reg [31:0] age;
 localparam integer CAL_WIDTH=$clog2(CALIBRATION_SAMPLES);
 localparam integer SUM_WIDTH=CAL_WIDTH+10;
 reg [CAL_WIDTH-1:0] calibration_count;
 reg signed [SUM_WIDTH-1:0] sum_x,sum_y,sum_z;
 reg signed [15:0] bias_x,bias_y,bias_z;
 integer track_count;
 reg active_previous;
 reg signed [31:0] yaw_q16,px,py,pz,vx,vy,vz;
 integer yaw_index,roll_rad,pitch_rad;
 integer sr,cr,sp,cp,sy,cy,u,v,w,fx,fy,fz,wx,wy,wz,ax_mm,ay_mm,az_mm;
 reg stationary;
 reg signed [63:0] nvx,nvy,nvz,npx,npy,npz,nyaw;
 // Diagnostic yaw resolution 2.8125 degrees, nearest-bin lookup.
 function integer yaw_sine;
  input [6:0] index;
  begin case(index)
   0: yaw_sine=0;
   1: yaw_sine=1608;
   2: yaw_sine=3212;
   3: yaw_sine=4808;
   4: yaw_sine=6393;
   5: yaw_sine=7962;
   6: yaw_sine=9512;
   7: yaw_sine=11039;
   8: yaw_sine=12540;
   9: yaw_sine=14010;
   10: yaw_sine=15447;
   11: yaw_sine=16846;
   12: yaw_sine=18205;
   13: yaw_sine=19520;
   14: yaw_sine=20788;
   15: yaw_sine=22006;
   16: yaw_sine=23170;
   17: yaw_sine=24279;
   18: yaw_sine=25330;
   19: yaw_sine=26320;
   20: yaw_sine=27246;
   21: yaw_sine=28106;
   22: yaw_sine=28899;
   23: yaw_sine=29622;
   24: yaw_sine=30274;
   25: yaw_sine=30853;
   26: yaw_sine=31357;
   27: yaw_sine=31786;
   28: yaw_sine=32138;
   29: yaw_sine=32413;
   30: yaw_sine=32610;
   31: yaw_sine=32729;
   32: yaw_sine=32768;
   33: yaw_sine=32729;
   34: yaw_sine=32610;
   35: yaw_sine=32413;
   36: yaw_sine=32138;
   37: yaw_sine=31786;
   38: yaw_sine=31357;
   39: yaw_sine=30853;
   40: yaw_sine=30274;
   41: yaw_sine=29622;
   42: yaw_sine=28899;
   43: yaw_sine=28106;
   44: yaw_sine=27246;
   45: yaw_sine=26320;
   46: yaw_sine=25330;
   47: yaw_sine=24279;
   48: yaw_sine=23170;
   49: yaw_sine=22006;
   50: yaw_sine=20788;
   51: yaw_sine=19520;
   52: yaw_sine=18205;
   53: yaw_sine=16846;
   54: yaw_sine=15447;
   55: yaw_sine=14010;
   56: yaw_sine=12540;
   57: yaw_sine=11039;
   58: yaw_sine=9512;
   59: yaw_sine=7962;
   60: yaw_sine=6393;
   61: yaw_sine=4808;
   62: yaw_sine=3212;
   63: yaw_sine=1608;
   64: yaw_sine=0;
   65: yaw_sine=-1608;
   66: yaw_sine=-3212;
   67: yaw_sine=-4808;
   68: yaw_sine=-6393;
   69: yaw_sine=-7962;
   70: yaw_sine=-9512;
   71: yaw_sine=-11039;
   72: yaw_sine=-12540;
   73: yaw_sine=-14010;
   74: yaw_sine=-15447;
   75: yaw_sine=-16846;
   76: yaw_sine=-18205;
   77: yaw_sine=-19520;
   78: yaw_sine=-20788;
   79: yaw_sine=-22006;
   80: yaw_sine=-23170;
   81: yaw_sine=-24279;
   82: yaw_sine=-25330;
   83: yaw_sine=-26320;
   84: yaw_sine=-27246;
   85: yaw_sine=-28106;
   86: yaw_sine=-28899;
   87: yaw_sine=-29622;
   88: yaw_sine=-30274;
   89: yaw_sine=-30853;
   90: yaw_sine=-31357;
   91: yaw_sine=-31786;
   92: yaw_sine=-32138;
   93: yaw_sine=-32413;
   94: yaw_sine=-32610;
   95: yaw_sine=-32729;
   96: yaw_sine=-32768;
   97: yaw_sine=-32729;
   98: yaw_sine=-32610;
   99: yaw_sine=-32413;
   100: yaw_sine=-32138;
   101: yaw_sine=-31786;
   102: yaw_sine=-31357;
   103: yaw_sine=-30853;
   104: yaw_sine=-30274;
   105: yaw_sine=-29622;
   106: yaw_sine=-28899;
   107: yaw_sine=-28106;
   108: yaw_sine=-27246;
   109: yaw_sine=-26320;
   110: yaw_sine=-25330;
   111: yaw_sine=-24279;
   112: yaw_sine=-23170;
   113: yaw_sine=-22006;
   114: yaw_sine=-20788;
   115: yaw_sine=-19520;
   116: yaw_sine=-18205;
   117: yaw_sine=-16846;
   118: yaw_sine=-15447;
   119: yaw_sine=-14010;
   120: yaw_sine=-12540;
   121: yaw_sine=-11039;
   122: yaw_sine=-9512;
   123: yaw_sine=-7962;
   124: yaw_sine=-6393;
   125: yaw_sine=-4808;
   126: yaw_sine=-3212;
   127: yaw_sine=-1608;
   default:yaw_sine=0;
  endcase end
 endfunction
 function integer mul_q15;
  input integer a,b;
  reg signed [63:0] product;
  begin product=$signed(a)*$signed(b);mul_q15=product/32768;end
 endfunction
 assign estimate_valid=calibrated && enable && imu_valid && !degraded && age<PERIOD*2 && !rst;
 always @* begin
  // sin(theta) ~ theta, cos(theta) ~ 1-theta^2/2; diagnostic envelope +/-15deg.
  roll_rad=$signed(roll_cd)*32768/5730;pitch_rad=$signed(pitch_cd)*32768/5730;
  sr=roll_rad;cr=32768-mul_q15(roll_rad,roll_rad)/2;
  sp=pitch_rad;cp=32768-mul_q15(pitch_rad,pitch_rad)/2;
  yaw_index=((yaw_q16/65536+18000)*128+18000)/36000;
  sy=yaw_sine((yaw_index+64)%128);cy=yaw_sine((yaw_index+96)%128);
  u=accel_x;v=mul_q15(cr,accel_y)-mul_q15(sr,accel_z);
  w=mul_q15(sr,accel_y)+mul_q15(cr,accel_z);
  fx=mul_q15(cp,u)-mul_q15(sp,w);fy=v;fz=mul_q15(sp,u)+mul_q15(cp,w);
  wx=mul_q15(cy,fx)+mul_q15(sy,fy);wy=-mul_q15(sy,fx)+mul_q15(cy,fy);wz=fz-16384;
  ax_mm=wx*9807/16384;ay_mm=wy*9807/16384;az_mm=wz*9807/16384;
  stationary=ax_mm>-300 && ax_mm<300 && ay_mm>-300 && ay_mm<300 && az_mm>-300 && az_mm<300 &&
   yaw_rate_cdps>-100 && yaw_rate_cdps<100;
  nvx=vx+$signed(ax_mm-bias_x)*DT_Q16;
  nvy=vy+$signed(ay_mm-bias_y)*DT_Q16;
  nvz=TRACK_Z ? vz+$signed(az_mm-bias_z)*DT_Q16 : 0;
  npx=px+(nvx*DT_Q16)/65536;npy=py+(nvy*DT_Q16)/65536;npz=TRACK_Z ? pz+(nvz*DT_Q16)/65536 : 0;
  nyaw=yaw_q16+$signed(yaw_rate_cdps)*DT_Q16;
  if(nyaw>=18000*64'sd65536)nyaw=nyaw-36000*64'sd65536;
  if(nyaw< -18000*64'sd65536)nyaw=nyaw+36000*64'sd65536;
 end
 always @(posedge clk) begin
  if(rst) begin
   age<=PERIOD*2;calibrated<=0;degraded<=0;calibration_count<=0;
   sum_x<=0;sum_y<=0;sum_z<=0;bias_x<=0;bias_y<=0;bias_z<=0;
   active_previous<=0;track_count<=0;yaw_q16<=0;px<=0;py<=0;pz<=0;vx<=0;vy<=0;vz<=0;
   x_mm<=0;y_mm<=0;z_mm<=0;vx_mmps<=0;vy_mmps<=0;vz_mmps<=0;
  end else begin
   active_previous<=enable;
   if(age<PERIOD*2)age<=age+1'b1;
   if(!enable)begin
    degraded<=0;track_count<=0;yaw_q16<=0;px<=0;py<=0;pz<=0;vx<=0;vy<=0;vz<=0;
    x_mm<=0;y_mm<=0;z_mm<=0;vx_mmps<=0;vy_mmps<=0;vz_mmps<=0;
   end
   if(enable && (!imu_valid || accel_clipped || age>=PERIOD*2 ||
     roll_cd>1500 || roll_cd< -1500 || pitch_cd>1500 || pitch_cd< -1500))degraded<=1;
   if(sample_valid && imu_valid)begin
    age<=0;
    if(!calibrated && !enable)begin
     if(!stationary || accel_clipped)begin calibration_count<=0;sum_x<=0;sum_y<=0;sum_z<=0;end
     else if(calibration_count==CALIBRATION_SAMPLES-1)begin
      bias_x<=(sum_x+ax_mm)/CALIBRATION_SAMPLES;bias_y<=(sum_y+ay_mm)/CALIBRATION_SAMPLES;
      bias_z<=(sum_z+az_mm)/CALIBRATION_SAMPLES;calibrated<=1;
     end else begin calibration_count<=calibration_count+1;sum_x<=sum_x+ax_mm;sum_y<=sum_y+ay_mm;sum_z<=sum_z+az_mm;end
    end
    if(enable && active_previous && calibrated && !degraded && !accel_clipped)begin
     if(age<PERIOD/2 || age>PERIOD*3/2 || track_count>=SAMPLE_HZ*MAX_TRACK_SECONDS ||
        nvx>10000*64'sd65536 || nvx< -10000*64'sd65536 || nvy>10000*64'sd65536 || nvy< -10000*64'sd65536 ||
        nvz>10000*64'sd65536 || nvz< -10000*64'sd65536 || npx>10000*64'sd65536 || npx< -10000*64'sd65536 ||
        npy>10000*64'sd65536 || npy< -10000*64'sd65536 || npz>10000*64'sd65536 || npz< -10000*64'sd65536)degraded<=1;
     else begin
      track_count<=track_count+1;yaw_q16<=nyaw;vx<=nvx;vy<=nvy;vz<=nvz;px<=npx;py<=npy;pz<=npz;
      x_mm<=npx/65536;y_mm<=npy/65536;z_mm<=npz/65536;
      vx_mmps<=nvx/65536;vy_mmps<=nvy/65536;vz_mmps<=nvz/65536;
     end
    end
   end
  end
 end
 // synthesis translate_off
 initial if(CLK_FREQ<1000 || SAMPLE_HZ<1 || PERIOD<4 || CALIBRATION_SAMPLES<2 || CALIBRATION_SAMPLES>4096 || MAX_TRACK_SECONDS<1 || (TRACK_Z!=0 && TRACK_Z!=1))
  $fatal(1,"PositionEstimator: invalid parameters");
 // synthesis translate_on
endmodule

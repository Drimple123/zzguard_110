package freechips.rocketchip.zzguardrr

import chisel3._
import chisel3.util._
//import freechips.rocketchip.tile.ClockDividerN
//lht start
import freechips.rocketchip.tile._
import freechips.rocketchip.diplomacy._
import org.chipsalliance.cde.config.Parameters
//lht end
class zzguardrr_ram_new(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new zzguardrr_ramImp_new (this)
}

class zzguardrr_ramImp_new(outer: zzguardrr_ram_new)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
    with HasCoreParameters {
  //val io = IO(new Bundle{
    //val addr        =   Input(UInt(2.W))
    val valid       =   Wire(Bool())
    val din_pc      =   Wire(UInt(40.W))
    val din_ins     =   Wire(UInt(32.W))
    val din_wdata   =   Wire(UInt(64.W))
    val din_mdata   =   Wire(UInt(64.W))
    val din_npc     =   Wire(UInt(40.W))
    val din_req_addr=   Wire(UInt(40.W))

    val cmd                     = io.cmd
    val funct                   = cmd.bits.inst.funct
    val rs2                     = cmd.bits.inst.rs2
    val rs1                     = cmd.bits.inst.rs1
    val xd                      = cmd.bits.inst.xd
    val xs1                     = cmd.bits.inst.xs1
    val xs2                     = cmd.bits.inst.xs2
    val rd                      = cmd.bits.inst.rd
    val opcode                  = cmd.bits.inst.opcode

    val rs1_val                 = cmd.bits.rs1
    val rs2_val                 = cmd.bits.rs2
    val rd_val                  = WireInit(0.U(xLen.W))
  dontTouch(cmd)  
  dontTouch(io)

  val rocc_packet = Wire(UInt(55.W))
  val packet_mid  = Wire(UInt(48.W))
  packet_mid := Cat(cmd.bits.rs1(39,0), cmd.bits.rs2(7,0))
  rocc_packet := Cat(packet_mid, cmd.bits.inst.funct)
  //val q_rocc = Module(new fifox(55, 32, 10))
  val q_rocc = VecInit(Seq.fill(3)(Module(new fifox(55, 32, 10)).io))
  for(i <- 0 to 2){
    q_rocc(i).in.bits := rocc_packet
    io.asan_io.get(i) <> q_rocc(i).out
  }
  
  





  cmd.ready                  := true.B
  io.resp.bits.rd            := cmd.bits.inst.rd
  io.resp.valid              := cmd.valid
  //io.resp.bits.data          := counter.io.number_load
  io.busy                    := cmd.valid
  
  //mask,写表之前为0,写完表置1,程序运行完之后置0
  val cfg_mask = RegInit(0.U)

  valid       := io.valid.get & cfg_mask
  din_pc      := io.pc.get & Fill(40, cfg_mask)
  din_ins     := io.ins.get & Fill(32, cfg_mask)
  din_wdata   := io.wdata.get & Fill(64, cfg_mask)
  din_mdata   := io.mdata.get & Fill(64, cfg_mask)
  din_npc     := io.mem_npc.get & Fill(40, cfg_mask)
  din_req_addr:= io.req_addr.get & Fill(40, cfg_mask)
  
  //因为查表控制信号慢了1拍，所以数据也慢1拍
  val ins_r   = RegNext(din_ins,0.U)
  val wdata_r = RegNext(din_wdata,0.U)
  val mdata_r = RegNext(din_mdata,0.U)
  val valid_r = RegNext(valid,false.B)
  val npc_r = RegNext(din_npc, 0.U)
  val req_addr_r= RegNext(din_req_addr, 0.U)

  val table = Module(new look_2table_ram(4))
  table.io.ren1 := valid
  table.io.ren2 := valid_r

  table.io.opcode   := din_ins(6,0)
  table.io.addr1    := rs1_val
  table.io.addr2    := rs1_val
  table.io.data_in1 := rs2_val
  table.io.data_in2 := rs2_val

  when(cmd.fire()){
    when((funct === 6.U)){//传到另一个核的asan
      q_rocc(0).in.valid := true.B
      q_rocc(1).in.valid := true.B
      q_rocc(2).in.valid := true.B
      table.io.wen1 := false.B
      table.io.wen2 := false.B
    }
    .elsewhen(funct === 4.U){//写表完成，开始检测
      q_rocc(0).in.valid := false.B
      q_rocc(1).in.valid := false.B
      q_rocc(2).in.valid := false.B
      cfg_mask := 1.U
      table.io.wen1 := false.B
      table.io.wen2 := false.B
    }
    .elsewhen(funct === 8.U){//主要程序跑完，结束检测
      q_rocc(0).in.valid := false.B
      q_rocc(1).in.valid := false.B
      q_rocc(2).in.valid := false.B
      cfg_mask := 0.U
      table.io.wen1 := false.B
      table.io.wen2 := false.B
    }
    .elsewhen(funct === 1.U){  //写第一张表
      q_rocc(0).in.valid := false.B
      q_rocc(1).in.valid := false.B
      q_rocc(2).in.valid := false.B
      table.io.wen1 := true.B
      table.io.wen2 := false.B
    }
    .elsewhen(funct === 2.U){  //写第二张表
      q_rocc(0).in.valid := false.B
      q_rocc(1).in.valid := false.B
      q_rocc(2).in.valid := false.B
      table.io.wen1 := false.B
      table.io.wen2 := true.B
    }
    .otherwise{
      q_rocc(0).in.valid := false.B
      q_rocc(1).in.valid := false.B
      q_rocc(2).in.valid := false.B
      table.io.wen1 := false.B
      table.io.wen2 := false.B
    }
  }
  .otherwise{
    q_rocc(0).in.valid := false.B
    q_rocc(1).in.valid := false.B
    q_rocc(2).in.valid := false.B
    table.io.wen1 := false.B
    table.io.wen2 := false.B
  }




  val bitmap = WireDefault(0.U(4.W))
  bitmap := table.io.bitmap

  val cat = Module(new instruction_cat1)
  //cat.io.in_1  := io.din_pc
  cat.io.ins    := ins_r
  cat.io.wdata  := wdata_r
  cat.io.mdata  := mdata_r
  cat.io.npc    := npc_r
  cat.io.req_addr := req_addr_r

  cat.io.sel    := table.io.sel
  


  //val q = VecInit(Seq.fill(2)(Module(new asyncfifo(16, 160)).io))
  //q0是ss,q1是counter，q2是asan0，q3是rowhammer，q4和q5是asan1和asan2,q6和q7是counter1和2
  val q = VecInit(Seq.fill(17)(Module(new fifox(160, 32, 10)).io))
  val q_full_counter = RegInit(VecInit(Seq.fill(17)(0.U(32.W))))
  dontTouch(q_full_counter)
  for(i <- 0 to 16){
    when(q(i).count === 28.U){
      q_full_counter(i) := q_full_counter(i) + 1.U
    }
  }

  //val yao_full = VecInit(Seq.fill(17)(Wire(Bool())))
  val yao_full = WireDefault(VecInit(Seq.fill(17)(false.B)))
  for(i <- 0 until 17){
    when(q(i).count >= 28.U){
      yao_full(i) := true.B
    }
    .otherwise{
      yao_full(i) := false.B
    }
  }
  val yao_full_25 = WireDefault(VecInit(Seq.fill(17)(false.B)))
  for(i <- 0 until 17){
    when(q(i).count >= 25.U){
      yao_full_25(i) := true.B
    }
    .otherwise{
      yao_full_25(i) := false.B
    }
  }
  val fifo_empty = WireDefault(VecInit(Seq.fill(17)(true.B)))
  for(i <- 0 until 17){
    when(q(i).count === 0.U){
      fifo_empty(i) := true.B
    }
    .otherwise{
      fifo_empty(i) := false.B
    }
  }
  dontTouch(yao_full)
  dontTouch(fifo_empty)
  dontTouch(yao_full_25)
  val yao_full_01234 = yao_full(0) || yao_full(1) || yao_full(2) || yao_full(3) || yao_full(4)
  val yao_full_5678 = yao_full(5) || yao_full(6) || yao_full(7) || yao_full(8)
  val yao_full_9_12 = yao_full(9) || yao_full(10) || yao_full(11) || yao_full(12)
  val yao_full_13_16 = yao_full(13) || yao_full(14) || yao_full(15) || yao_full(16)
  io.fifo_full.get := yao_full_01234 || yao_full_5678 || yao_full_9_12 || yao_full_13_16
  //只要有一个不ready，就把主核stall住
  //io.fifo_ready.get := q(0).in.ready && q(1).in.ready && q(2).in.ready && q(3).in.ready && q(4).in.ready && q(5).in.ready && q(6).in.ready && q(7).in.ready && q(8).in.ready && q(9).in.ready && q(10).in.ready && q(11).in.ready && q(12).in.ready && q(13).in.ready && q(14).in.ready && q(15).in.ready && q(16).in.ready

  for(i<- List(0,3)){
    q(i).in.bits := cat.io.out
    //q(i).out.ready := io.fifo_io(i).ready
    when(valid_r){
      when(bitmap(i) === 1.U){
        q(i).in.valid := true.B
      }
      .otherwise{
        q(i).in.valid := false.B
      }
    }
    .otherwise{
      q(i).in.valid := false.B
    }
    io.fifo_io.get(i) <> q(i).out
    dontTouch(q(i).count)
  }
  
  for(i <- List(1,2,4,5,6,7,8,9,10,11,12,13,14,15,16)){
    q(i).in.bits := cat.io.out
  }
  //q(2).out.ready := io.fifo_io(2).ready
  // when(valid_r){
  //   when(bitmap(2) === 1.U){
  //     when(mdata_r >= "h88000000".U && mdata_r <="h88100000".U){
  //       q(2).in.valid := true.B
  //     }
  //     .otherwise{
  //       q(2).in.valid := false.B
  //     }
  //   }
  //   .otherwise{
  //     q(2).in.valid := false.B
  //   }
  // }
  // .otherwise{
  //   q(2).in.valid := false.B
  // }


  val rr_asan = Module(new fsm_rr_seq(Seq(2,4,5,11,12,13,14,15,16)))
  // for((i,j) <- List((0,2),(1,4))){
  //   rr_asan.io.a(i) := q(j).count
  // }
  val rr_counter= Module(new fsm_rr_seq(Seq(1,6,7,9)))
  dontTouch(q(4))
  dontTouch(q(5))
  dontTouch(q(6))
  dontTouch(q(7))
  dontTouch(q(8))
  dontTouch(q(9))
  dontTouch(q(10))
  // q(1).out.ready := true.B
  // q(6).out.ready := true.B
  // q(7).out.ready := true.B
  //asan的处理
  when(valid_r){
    when(bitmap(2) === 1.U){
        rr_asan.io.en := true.B
        for(i<- List(2,4,5,8,11,12,13,14,15,16)){
          when(rr_asan.io.num === i.U){
            q(i).in.valid := true.B
          }
          .otherwise{
            q(i).in.valid := false.B
          }
        }
    }
    .otherwise{
      rr_asan.io.en := false.B
      q(2).in.valid := false.B
      q(4).in.valid := false.B
      q(5).in.valid := false.B
      q(8).in.valid := false.B
      q(11).in.valid := false.B
      q(12).in.valid := false.B
      q(13).in.valid := false.B
      q(14).in.valid := false.B
      q(15).in.valid := false.B
      q(16).in.valid := false.B

    }
  }
  .otherwise{
    rr_asan.io.en := false.B
    q(2).in.valid := false.B
    q(4).in.valid := false.B
    q(5).in.valid := false.B
    q(8).in.valid := false.B
    q(11).in.valid := false.B
    q(12).in.valid := false.B
    q(13).in.valid := false.B
    q(14).in.valid := false.B
    q(15).in.valid := false.B
    q(16).in.valid := false.B
    
  }

  for(i<- List(1,2,4,5,6,7,8,9,10,11,12,13,14,15,16)){
    io.fifo_io.get(i) <> q(i).out
  }

  //搞counter
  when(valid_r){
    when(bitmap(1) === 1.U){
      rr_counter.io.en := true.B
      for(i<- List(1,6,7,9,10)){
        when(rr_counter.io.num === i.U){
          q(i).in.valid := true.B
        }
        .otherwise{
          q(i).in.valid := false.B
        }
      }
    }
    .otherwise{
      rr_counter.io.en := false.B
      q(1).in.valid := false.B
      q(6).in.valid := false.B
      q(7).in.valid := false.B
      q(9).in.valid := false.B
      q(10).in.valid := false.B
    }
  }
  .otherwise{
    rr_counter.io.en := false.B
    q(1).in.valid := false.B
    q(6).in.valid := false.B
    q(7).in.valid := false.B
    q(9).in.valid := false.B
    q(10).in.valid := false.B
    
  }
  dontTouch(q(2).count)


  //看前满后满
  // val qmhm_counter = RegInit(VecInit(Seq.fill(3)(0.U(15.W))))
  // val qmhbm_counter = RegInit(VecInit(Seq.fill(3)(0.U(15.W))))
  // dontTouch(qmhm_counter)
  // dontTouch(qmhbm_counter)
  // for((i,j) <- List((2,0), (4,1), (5,2))){
  //   when(q(i).count === 32.U && io.count_io.get(j) === 32.U){
  //     qmhm_counter(j) := qmhm_counter(j) + 1.U
  //   }
  //   when(q(i).count === 32.U && io.count_io.get(j) =/= 32.U){
  //     qmhm_counter(j) := qmhm_counter(j) + 1.U
  //   }
  // }
}


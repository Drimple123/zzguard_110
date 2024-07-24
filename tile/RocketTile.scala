// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.

package freechips.rocketchip.tile

//import Chisel._
// import chisel3._
// import chisel3.util._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.rocket._
import freechips.rocketchip.subsystem.TileCrossingParamsLike
import freechips.rocketchip.util._
import freechips.rocketchip.prci.{ClockSinkParameters}
import freechips.rocketchip.zzguardrr._

case class RocketTileBoundaryBufferParams(force: Boolean = false)

case class RocketTileParams(
    core: RocketCoreParams = RocketCoreParams(),
    icache: Option[ICacheParams] = Some(ICacheParams()),
    dcache: Option[DCacheParams] = Some(DCacheParams()),
    btb: Option[BTBParams] = Some(BTBParams()),
    dataScratchpadBytes: Int = 0,
    name: Option[String] = Some("tile"),
    hartId: Int = 0,
    beuAddr: Option[BigInt] = None,
    blockerCtrlAddr: Option[BigInt] = None,
    clockSinkParams: ClockSinkParameters = ClockSinkParameters(),
    boundaryBuffers: Option[RocketTileBoundaryBufferParams] = None
    ) extends InstantiableTileParams[RocketTile] {
  require(icache.isDefined)
  require(dcache.isDefined)
  def instantiate(crossing: TileCrossingParamsLike, lookup: LookupByHartIdImpl)(implicit p: Parameters): RocketTile = {
    new RocketTile(this, crossing, lookup)
  }
}

class RocketTile private(
      val rocketParams: RocketTileParams,
      crossing: ClockCrossingType,
      lookup: LookupByHartIdImpl,
      q: Parameters)
    extends BaseTile(rocketParams, crossing, lookup, q)
    with SinksExternalInterrupts
    with SourcesExternalNotifications
    with HasLazyRoCC  // implies CanHaveSharedFPU with CanHavePTW with HasHellaCache
    with HasHellaCache
    with HasICacheFrontend
{
  // Private constructor ensures altered LazyModule.p is used implicitly
  def this(params: RocketTileParams, crossing: TileCrossingParamsLike, lookup: LookupByHartIdImpl)(implicit p: Parameters) =
    this(params, crossing.crossingType, lookup, p)

  val intOutwardNode = IntIdentityNode()
  val slaveNode = TLIdentityNode()
  val masterNode = visibilityNode

  val dtim_adapter = tileParams.dcache.flatMap { d => d.scratch.map { s =>
    LazyModule(new ScratchpadSlavePort(AddressSet.misaligned(s, d.dataScratchpadBytes), lazyCoreParamsView.coreDataBytes, tileParams.core.useAtomics && !tileParams.core.useAtomicsOnlyForIO))
  }}
  dtim_adapter.foreach(lm => connectTLSlave(lm.node, lm.node.portParams.head.beatBytes))

  val bus_error_unit = rocketParams.beuAddr map { a =>
    val beu = LazyModule(new BusErrorUnit(new L1BusErrors, BusErrorUnitParams(a)))
    intOutwardNode := beu.intNode
    connectTLSlave(beu.node, xBytes)
    beu
  }

  val tile_master_blocker =
    tileParams.blockerCtrlAddr
      .map(BasicBusBlockerParams(_, xBytes, masterPortBeatBytes, deadlock = true))
      .map(bp => LazyModule(new BasicBusBlocker(bp)))

  tile_master_blocker.foreach(lm => connectTLSlave(lm.controlNode, xBytes))

  // TODO: this doesn't block other masters, e.g. RoCCs
  tlOtherMastersNode := tile_master_blocker.map { _.node := tlMasterXbar.node } getOrElse { tlMasterXbar.node }
  masterNode :=* tlOtherMastersNode
  DisableMonitors { implicit p => tlSlaveXbar.node :*= slaveNode }

  nDCachePorts += 1 /*core */ + (dtim_adapter.isDefined).toInt

  val dtimProperty = dtim_adapter.map(d => Map(
    "sifive,dtim" -> d.device.asProperty)).getOrElse(Nil)

  val itimProperty = frontend.icache.itimProperty.toSeq.flatMap(p => Map("sifive,itim" -> p))

  val beuProperty = bus_error_unit.map(d => Map(
          "sifive,buserror" -> d.device.asProperty)).getOrElse(Nil)

  val cpuDevice: SimpleDevice = new SimpleDevice("cpu", Seq("sifive,rocket0", "riscv")) {
    override def parent = Some(ResourceAnchors.cpus)
    override def describe(resources: ResourceBindings): Description = {
      val Description(name, mapping) = super.describe(resources)
      Description(name, mapping ++ cpuProperties ++ nextLevelCacheProperty
                  ++ tileProperties ++ dtimProperty ++ itimProperty ++ beuProperty)
    }
  }

  ResourceBinding {
    Resource(cpuDevice, "reg").bind(ResourceAddress(staticIdForMetadataUseOnly))
  }

  override lazy val module = new RocketTileModuleImp(this)

  override def makeMasterBoundaryBuffers(crossing: ClockCrossingType)(implicit p: Parameters) = (rocketParams.boundaryBuffers, crossing) match {
    case (Some(RocketTileBoundaryBufferParams(true )), _)                   => TLBuffer()
    case (Some(RocketTileBoundaryBufferParams(false)), _: RationalCrossing) => TLBuffer(BufferParams.none, BufferParams.flow, BufferParams.none, BufferParams.flow, BufferParams(1))
    case _ => TLBuffer(BufferParams.none)
  }

  override def makeSlaveBoundaryBuffers(crossing: ClockCrossingType)(implicit p: Parameters) = (rocketParams.boundaryBuffers, crossing) match {
    case (Some(RocketTileBoundaryBufferParams(true )), _)                   => TLBuffer()
    case (Some(RocketTileBoundaryBufferParams(false)), _: RationalCrossing) => TLBuffer(BufferParams.flow, BufferParams.none, BufferParams.none, BufferParams.none, BufferParams.none)
    case _ => TLBuffer(BufferParams.none)
  }
}

class RocketTileModuleImp(outer: RocketTile) extends BaseTileModuleImp(outer)
    with HasFpuOpt
    with HasLazyRoCCModule
    with HasICacheFrontendModule {
  Annotated.params(this, outer.rocketParams)

  val core = Module(new Rocket(outer)(outer.p))
  

  if(outer.rocketParams.hartId == 0){
    //println("######zzguard###########   hartId: ",outer.rocketParams.hartId,"  ############")

    for(i<-0 to 10){
      outer.data_bits_out_nodes.get(i).bundle:= outer.roccs(0).module.io.fifo_io.get(i).bits
      outer.data_valid_out_nodes.get(i).bundle:= outer.roccs(0).module.io.fifo_io.get(i).valid
      outer.roccs(0).module.io.fifo_io.get(i).ready := outer.data_ready_in_nodes.get(i).bundle

    }

    for((i,j) <- List((11,0),(12,1),(13,2))){
      outer.data_bits_out_nodes_2.get(j).bundle:= outer.roccs(0).module.io.fifo_io.get(i).bits
      outer.data_valid_out_nodes_2.get(j).bundle:= outer.roccs(0).module.io.fifo_io.get(i).valid
      outer.roccs(0).module.io.fifo_io.get(i).ready := outer.data_ready_in_nodes_2.get(j).bundle

    }

    for((i,j) <- List((14,0),(15,1),(16,2))){
      outer.data_bits_out_nodes_3.get(j).bundle:= outer.roccs(0).module.io.fifo_io.get(i).bits
      outer.data_valid_out_nodes_3.get(j).bundle:= outer.roccs(0).module.io.fifo_io.get(i).valid
      outer.roccs(0).module.io.fifo_io.get(i).ready := outer.data_ready_in_nodes_3.get(j).bundle

    }

    

    

    core.io.ready_stall.get := outer.roccs(0).module.io.fifo_full.get
    outer.rocc_bits_out.get.bundle := outer.roccs(0).module.io.asan_io.get(0).bits
    outer.rocc_valid_out.get.bundle := outer.roccs(0).module.io.asan_io.get(0).valid
    outer.roccs(0).module.io.asan_io.get(0).ready := outer.rocc_ready_in.get.bundle

    outer.rocc_bits_out_2.get.bundle := outer.roccs(0).module.io.asan_io.get(1).bits
    outer.rocc_valid_out_2.get.bundle := outer.roccs(0).module.io.asan_io.get(1).valid
    outer.roccs(0).module.io.asan_io.get(1).ready := outer.rocc_ready_in_2.get.bundle

    outer.rocc_bits_out_3.get.bundle := outer.roccs(0).module.io.asan_io.get(2).bits
    outer.rocc_valid_out_3.get.bundle := outer.roccs(0).module.io.asan_io.get(2).valid
    outer.roccs(0).module.io.asan_io.get(2).ready := outer.rocc_ready_in_3.get.bundle

    //新的直接从core里面拉到zzguard的一条路
    outer.roccs(0).module.io.valid.get := core.io.valid.get
    outer.roccs(0).module.io.pc.get := core.io.pc.get
    outer.roccs(0).module.io.ins.get := core.io.ins.get
    outer.roccs(0).module.io.wdata.get := core.io.wdata.get
    outer.roccs(0).module.io.mdata.get := core.io.mdata.get
    outer.roccs(0).module.io.mem_npc.get := core.io.mem_npc.get
    outer.roccs(0).module.io.req_addr.get := core.io.req_addr.get
    
  }
  else if(outer.rocketParams.hartId == 1){
    //println("######zzguard###########   hartId: ",outer.rocketParams.hartId,"  ############")

    val q = VecInit(Seq.fill(11)(Module(new Queue(UInt(160.W), 32)).io))
    val q_full_counter = RegInit(VecInit(Seq.fill(11)(0.U(32.W))))
    dontTouch(q_full_counter)
    for(i <- 0 to 10){
      when(q(i).count === 32.U){
        q_full_counter(i) := q_full_counter(i) + 1.U
      }
    }
    for(i<-0 to 10){
      dontTouch(q(i).deq)
    }
    for(i<-0 to 10){
      q(i).enq.bits := outer.data_bits_in_nodes.get(i).bundle
      q(i).enq.valid := outer.data_valid_in_nodes.get(i).bundle
      outer.data_ready_out_nodes.get(i).bundle := q(i).enq.ready
      dontTouch(q(i).count)
      //q(i).deq.ready := true.B
    }
    //q(3).deq.ready := true.B

    val q_rocc = Module(new Queue(UInt(55.W),16))
    q_rocc.io.enq.bits  := outer.rocc_bits_in.get.bundle
    q_rocc.io.enq.valid := outer.rocc_valid_in.get.bundle
    outer.rocc_ready_out.get.bundle := q_rocc.io.enq.ready
    q(3).deq.ready := true.B


    dontTouch(q_rocc.io)
    //接上counter_losuan
    //val counter_losuan_1 = Module(new counter_losuan)
    val counter_ls = VecInit(Seq.fill(5)(Module(new counter_losuan).io))
    //counter_losuan_1.io.enq <> q(1).deq
    counter_ls(0).enq <> q(1).deq
    counter_ls(1).enq <> q(6).deq
    counter_ls(2).enq <> q(7).deq
    counter_ls(3).enq <> q(9).deq
    counter_ls(4).enq <> q(10).deq

    
    //把3个counter的结果合起来
    val num_ls = counter_ls(0).number_losuan + counter_ls(1).number_losuan + counter_ls(2).number_losuan + counter_ls(3).number_losuan + counter_ls(4).number_losuan
    dontTouch(num_ls)

    //接上ss
    val ss = Module(new shadow_stack)
    ss.io.in <> q(0).deq

    outer.roccs(0).module.io.rocc_in.get <> q_rocc.io.deq
    outer.roccs(0).module.io.din.get <> q(2).deq

    outer.roccs(1).module.io.rocc_in.get <> q_rocc.io.deq
    outer.roccs(1).module.io.din.get <> q(4).deq

    outer.roccs(2).module.io.rocc_in.get <> q_rocc.io.deq
    outer.roccs(2).module.io.din.get <> q(5).deq

    outer.roccs(3).module.io.rocc_in.get <> q_rocc.io.deq
    outer.roccs(3).module.io.din.get <> q(8).deq

  }
  else if(outer.rocketParams.hartId == 2){
    val q = VecInit(Seq.fill(3)(Module(new Queue(UInt(160.W), 32)).io))
    val q_rocc = Module(new Queue(UInt(55.W),16))


    q_rocc.io.enq.bits  := outer.rocc_bits_in_2.get.bundle
    q_rocc.io.enq.valid := outer.rocc_valid_in_2.get.bundle
    outer.rocc_ready_out_2.get.bundle := q_rocc.io.enq.ready


    for(i<-0 to 2){
      q(i).enq.bits := outer.data_bits_in_nodes_2.get(i).bundle
      q(i).enq.valid := outer.data_valid_in_nodes_2.get(i).bundle
      outer.data_ready_out_nodes_2.get(i).bundle := q(i).enq.ready
      dontTouch(q(i).count)
      dontTouch(q(i).deq)
    }

    for(i <-0 to 2){
      outer.roccs(i).module.io.din.get <> q(i).deq
      outer.roccs(i).module.io.rocc_in.get <> q_rocc.io.deq
    }
  }
  else if(outer.rocketParams.hartId == 3){
    val q = VecInit(Seq.fill(3)(Module(new Queue(UInt(160.W), 32)).io))
    val q_rocc = Module(new Queue(UInt(55.W),16))


    q_rocc.io.enq.bits  := outer.rocc_bits_in_3.get.bundle
    q_rocc.io.enq.valid := outer.rocc_valid_in_3.get.bundle
    outer.rocc_ready_out_3.get.bundle := q_rocc.io.enq.ready


    for(i<-0 to 2){
      q(i).enq.bits := outer.data_bits_in_nodes_3.get(i).bundle
      q(i).enq.valid := outer.data_valid_in_nodes_3.get(i).bundle
      outer.data_ready_out_nodes_3.get(i).bundle := q(i).enq.ready
      dontTouch(q(i).count)
      dontTouch(q(i).deq)
    }

    for(i <-0 to 2){
      outer.roccs(i).module.io.din.get <> q(i).deq
      outer.roccs(i).module.io.rocc_in.get <> q_rocc.io.deq
    }
  }


  core.io.reset_vector := DontCare

  // Report unrecoverable error conditions; for now the only cause is cache ECC errors
  outer.reportHalt(List(outer.dcache.module.io.errors))

  // Report when the tile has ceased to retire instructions; for now the only cause is clock gating
  outer.reportCease(outer.rocketParams.core.clockGate.option(
    !outer.dcache.module.io.cpu.clock_enabled &&
    !outer.frontend.module.io.cpu.clock_enabled &&
    !ptw.io.dpath.clock_enabled &&
    core.io.cease))

  outer.reportWFI(Some(core.io.wfi))

  outer.decodeCoreInterrupts(core.io.interrupts) // Decode the interrupt vector

  outer.bus_error_unit.foreach { beu =>
    core.io.interrupts.buserror.get := beu.module.io.interrupt
    beu.module.io.errors.dcache := outer.dcache.module.io.errors
    beu.module.io.errors.icache := outer.frontend.module.io.errors
  }

  core.io.interrupts.nmi.foreach { nmi => nmi := outer.nmiSinkNode.bundle }

  // Pass through various external constants and reports that were bundle-bridged into the tile
  outer.traceSourceNode.bundle <> core.io.trace
  core.io.traceStall := outer.traceAuxSinkNode.bundle.stall
  outer.bpwatchSourceNode.bundle <> core.io.bpwatch
  core.io.hartid := outer.hartIdSinkNode.bundle
  require(core.io.hartid.getWidth >= outer.hartIdSinkNode.bundle.getWidth,
    s"core hartid wire (${core.io.hartid.getWidth}b) truncates external hartid wire (${outer.hartIdSinkNode.bundle.getWidth}b)")

  // Connect the core pipeline to other intra-tile modules
  outer.frontend.module.io.cpu <> core.io.imem
  dcachePorts += core.io.dmem // TODO outer.dcachePorts += () => module.core.io.dmem ??
  fpuOpt foreach { fpu => core.io.fpu <> fpu.io
    fpu.io.cp_req := DontCare
    fpu.io.cp_resp := DontCare
   }
  core.io.ptw <> ptw.io.dpath

  // Connect the coprocessor interfaces
  if (outer.roccs.size > 0) {
    cmdRouter.get.io.in <> core.io.rocc.cmd
    outer.roccs.foreach(_.module.io.exception := core.io.rocc.exception)
    core.io.rocc.resp <> respArb.get.io.out
    core.io.rocc.busy <> (cmdRouter.get.io.busy || outer.roccs.map(_.module.io.busy).reduce(_ || _))
    core.io.rocc.interrupt := outer.roccs.map(_.module.io.interrupt).reduce(_ || _)
    (core.io.rocc.csrs zip roccCSRIOs.flatten).foreach { t => t._2 := t._1 }
  }


  core.io.rocc.mem := DontCare

  // Rocket has higher priority to DTIM than other TileLink clients
  outer.dtim_adapter.foreach { lm => dcachePorts += lm.module.io.dmem }

  // TODO eliminate this redundancy
  val h = dcachePorts.size
  val c = core.dcacheArbPorts
  val o = outer.nDCachePorts
  require(h == c, s"port list size was $h, core expected $c")
  require(h == o, s"port list size was $h, outer counted $o")
  // TODO figure out how to move the below into their respective mix-ins
  dcacheArb.io.requestor <> dcachePorts.toSeq
  ptw.io.requestor <> ptwPorts.toSeq
}

trait HasFpuOpt { this: RocketTileModuleImp =>
  val fpuOpt = outer.tileParams.core.fpu.map(params => Module(new FPU(params)(outer.p)))
}

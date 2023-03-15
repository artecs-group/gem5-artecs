from m5.params import *

from m5.objects.Device import PioDevice

class SgaDmaDevice(PioDevice):
    type = 'SgaDmaDevice'
    cxx_header = "dev/sga_dma_device.hh"
    cxx_class = 'gem5::cat::SgaDmaDevice'
    abstract = True

    dma = RequestPort("DMA port")
    mmem_rowbuffer_size = Param.MemorySize(
        "Main memory page (row buffer) size")

    _iommu = None

    sid = Param.Unsigned(0,
        "Stream identifier used by an IOMMU to distinguish amongst "
        "several devices attached to it")
    ssid = Param.Unsigned(0,
        "Substream identifier used by an IOMMU to distinguish amongst "
        "several devices attached to it")

    def addIommuProperty(self, state, node):
        """
        This method takes an FdtState and a FdtNode as parameters, and
        it is appending a "iommus = <>" property in case the SgaDmaDevice
        is attached to an IOMMU.
        This method is necessary for autogenerating a binding between
        a dma device and the iommu.
        """
        if self._iommu is not None:
            node.append(FdtPropertyWords("iommus",
                [ state.phandle(self._iommu), self.sid ]))

class SgaDmaController(SgaDmaDevice):
    type = 'SgaDmaController'
    cxx_header = "dev/sga_dma_controller.hh"
    cxx_class = 'gem5::cat::SgaDmaController'

    pio_addr = Param.Addr("Device Address")
    pio_latency = Param.Cycles(1, "Programmed IO latency (cycles)")

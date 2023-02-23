from m5.params import *
from m5.objects.Device import BasicPioDevice

class CAT(BasicPioDevice):
    type = 'CAT'
    cxx_header = "dev/cat_translator.hh"
    cxx_class  = 'gem5::cat::CAT'
    entries    = Param.Int(8, "Number of CAT entries")
    config_lat = Param.Cycles(1, "Configuration latency (cycles)")
    start_lat  = Param.Cycles(4, "Translation start latency (cycles)")
    lookup_lat = Param.Cycles(0, "Address lookup latency (cycles)")

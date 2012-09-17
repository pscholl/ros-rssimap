from random import randint,sample
from select import select
from math import ceil
from sys import stdin,stdout,exit
from errno import errorcode
import struct,re,socket,atexit

# Flags values
NLM_F_REQUEST = 1       # It is a request message.
NLM_F_MULTI = 2         # Multipart message, terminated by NLMSG_DONE
NLM_F_ACK = 4           # Reply with ack, with zero or error code
NLM_F_ECHO = 8          # Echo this request

# flags in type value
NLA_F_NESTED = (1 << 15)
NLA_F_NET_BYTEORDER = (1 << 14)
NLA_F_MASK = (NLA_F_NET_BYTEORDER | NLA_F_NESTED)

# Modifiers to GET request
NLM_F_ROOT = 0x100      # specify tree root
NLM_F_MATCH = 0x200     # return all matching
NLM_F_ATOMIC = 0x400    # atomic GET
NLM_F_DUMP = (NLM_F_ROOT | NLM_F_MATCH)

# Modifiers to NEW request
NLM_F_REPLACE = 0x100   # Override existing
NLM_F_EXCL = 0x200      # Do not touch, if it exists
NLM_F_CREATE = 0x400    # Create, if it does not exist
NLM_F_APPEND = 0x800    # Add to end of list

# error values
NLMSG_NOOP = 0x1        # Nothing.
NLMSG_ERROR = 0x2       # Error
NLMSG_DONE = 0x3        # End of a dump
NLMSG_OVERRUN = 0x4     # Data lost
NLMSG_MIN_TYPE = 0x10

NLA_UNSPEC = 0
NLA_U8 = 1
NLA_U16 = 2
NLA_U32 = 3
NLA_U64 = 4
NLA_STRING = 5
NLA_FLAG = 6
NLA_MSECS = 7
NLA_NESTED = 8

NETLINK_ADD_MEMBERSHIP = 1
NETLINK_DROP_MEMBERSHIP = 2
NETLINK_PKTINFO = 3
NETLINK_BROADCAST_ERROR = 4
NETLINK_NO_ENOBUFS = 5
SOL_NETLINK = 270

#
# linux/genetlink.h
#
GENL_NAMSIZ = 16     # length of family name

GENL_MIN_ID = NLMSG_MIN_TYPE
GENL_MAX_ID = 1023

GENL_ADMIN_PERM = 0x01
GENL_CMD_CAP_DO = 0x02
GENL_CMD_CAP_DUMP = 0x04
GENL_CMD_CAP_HASPOL = 0x08

# List of reserved static generic netlink identifiers:
GENL_ID_GENERATE = 0
GENL_ID_CTRL = NLMSG_MIN_TYPE

# controller commands
CTRL_CMD_UNSPEC = 0
CTRL_CMD_NEWFAMILY = 1
CTRL_CMD_DELFAMILY = 2
CTRL_CMD_GETFAMILY = 3
CTRL_CMD_NEWOPS = 4
CTRL_CMD_DELOPS = 5
CTRL_CMD_GETOPS = 6
CTRL_CMD_NEWMCAST_GRP = 7
CTRL_CMD_DELMCAST_GRP = 8
CTRL_CMD_GETMCAST_GRP = 9     # unused
CTRL_CMD_MAX = 10             # always keep last

# generic netlink controller attribute types
CTRL_ATTR_UNSPEC = 0
CTRL_ATTR_FAMILY_ID = 1
CTRL_ATTR_FAMILY_NAME = 2
CTRL_ATTR_VERSION = 3
CTRL_ATTR_HDRSIZE = 4
CTRL_ATTR_MAXATTR = 5
CTRL_ATTR_OPS = 6
CTRL_ATTR_MCAST_GROUPS = 7
CTRL_ATTR_MAX = 8             # always keep last

CTRL_ATTR_OP_UNSPEC = 0
CTRL_ATTR_OP_ID = 1
CTRL_ATTR_OP_FLAGS = 2
CTRL_ATTR_OP_MAX = 3

CTRL_ATTR_MCAST_GRP_UNSPEC = 0
CTRL_ATTR_MCAST_GRP_NAME = 1
CTRL_ATTR_MCAST_GRP_ID = 2
CTRL_ATTR_MCAST_GRP_MAX = 3

# NetLink 802.11 defines
NL80211_CMD_UNSPEC = 0
NL80211_CMD_GET_WIPHY = 1
NL80211_CMD_SET_WIPHY = 2
NL80211_CMD_NEW_WIPHY = 3
NL80211_CMD_DEL_WIPHY = 4
NL80211_CMD_GET_INTERFACE = 5
NL80211_CMD_SET_INTERFACE = 6
NL80211_CMD_NEW_INTERFACE = 7
NL80211_CMD_DEL_INTERFACE = 8
NL80211_CMD_GET_KEY = 9
NL80211_CMD_SET_KEY = 10
NL80211_CMD_NEW_KEY = 11
NL80211_CMD_DEL_KEY = 12
NL80211_CMD_GET_BEACON = 13
NL80211_CMD_SET_BEACON = 14
NL80211_CMD_START_AP = 15
NL80211_CMD_NEW_BEACON = NL80211_CMD_START_AP
NL80211_CMD_STOP_AP = 16
NL80211_CMD_DEL_BEACON = NL80211_CMD_STOP_AP
NL80211_CMD_GET_STATION = 17
NL80211_CMD_SET_STATION = 18
NL80211_CMD_NEW_STATION = 19
NL80211_CMD_DEL_STATION = 20
NL80211_CMD_GET_MPATH = 21
NL80211_CMD_SET_MPATH = 22
NL80211_CMD_NEW_MPATH = 23
NL80211_CMD_DEL_MPATH = 24
NL80211_CMD_SET_BSS = 25
NL80211_CMD_SET_REG = 26
NL80211_CMD_REQ_SET_REG = 27
NL80211_CMD_GET_MESH_CONFIG = 28
NL80211_CMD_SET_MESH_CONFIG = 29
NL80211_CMD_SET_MGMT_EXTRA_IE = 30 # reserved; not used 
NL80211_CMD_GET_REG = 31
NL80211_CMD_GET_SCAN = 32
NL80211_CMD_TRIGGER_SCAN = 33
NL80211_CMD_NEW_SCAN_RESULTS = 34
NL80211_CMD_SCAN_ABORTED = 35
NL80211_CMD_REG_CHANGE = 36
NL80211_CMD_AUTHENTICATE = 37
NL80211_CMD_ASSOCIATE = 38
NL80211_CMD_DEAUTHENTICATE = 39
NL80211_CMD_DISASSOCIATE = 40
NL80211_CMD_MICHAEL_MIC_FAILURE = 41
NL80211_CMD_REG_BEACON_HINT = 42
NL80211_CMD_JOIN_IBSS = 43
NL80211_CMD_LEAVE_IBSS = 44
NL80211_CMD_TESTMODE = 45
NL80211_CMD_CONNECT = 46
NL80211_CMD_ROAM = 47
NL80211_CMD_DISCONNECT = 48
NL80211_CMD_SET_WIPHY_NETNS = 49
NL80211_CMD_GET_SURVEY = 50
NL80211_CMD_NEW_SURVEY_RESULTS = 51
NL80211_CMD_SET_PMKSA = 52
NL80211_CMD_DEL_PMKSA = 53
NL80211_CMD_FLUSH_PMKSA = 54
NL80211_CMD_REMAIN_ON_CHANNEL = 55
NL80211_CMD_CANCEL_REMAIN_ON_CHANNEL = 56
NL80211_CMD_SET_TX_BITRATE_MASK = 57
NL80211_CMD_REGISTER_FRAME = 58
NL80211_CMD_REGISTER_ACTION = NL80211_CMD_REGISTER_FRAME
NL80211_CMD_FRAME = 59
NL80211_CMD_ACTION = NL80211_CMD_FRAME
NL80211_CMD_FRAME_TX_STATUS = 60
NL80211_CMD_ACTION_TX_STATUS = NL80211_CMD_FRAME_TX_STATUS
NL80211_CMD_SET_POWER_SAVE = 61
NL80211_CMD_GET_POWER_SAVE = 62
NL80211_CMD_SET_CQM = 63
NL80211_CMD_NOTIFY_CQM = 64
NL80211_CMD_SET_CHANNEL = 65
NL80211_CMD_SET_WDS_PEER = 66
NL80211_CMD_FRAME_WAIT_CANCEL = 67
NL80211_CMD_JOIN_MESH = 68
NL80211_CMD_LEAVE_MESH = 69
NL80211_CMD_UNPROT_DEAUTHENTICATE = 70
NL80211_CMD_UNPROT_DISASSOCIATE = 71
NL80211_CMD_NEW_PEER_CANDIDATE = 72
NL80211_CMD_GET_WOWLAN = 73
NL80211_CMD_SET_WOWLAN = 74
NL80211_CMD_START_SCHED_SCAN = 75
NL80211_CMD_STOP_SCHED_SCAN = 76
NL80211_CMD_SCHED_SCAN_RESULTS = 77
NL80211_CMD_SCHED_SCAN_STOPPED = 78
NL80211_CMD_SET_REKEY_OFFLOAD = 79
NL80211_CMD_PMKSA_CANDIDATE = 80
NL80211_CMD_TDLS_OPER = 81
NL80211_CMD_TDLS_MGMT = 82
NL80211_CMD_UNEXPECTED_FRAME = 83
NL80211_CMD_PROBE_CLIENT = 84
NL80211_CMD_REGISTER_BEACONS = 85
NL80211_CMD_UNEXPECTED_4ADDR_FRAME = 86
NL80211_CMD_SET_NOACK_MAP = 87
NL80211_CMD_CH_SWITCH_NOTIFY = 88

NL80211_ATTR_UNSPEC = 0
NL80211_ATTR_WIPHY = 1
NL80211_ATTR_WIPHY_NAME = 2
NL80211_ATTR_IFINDEX = 3
NL80211_ATTR_IFNAME = 4
NL80211_ATTR_IFTYPE = 5
NL80211_ATTR_MAC = 6
NL80211_ATTR_KEY_DATA = 7
NL80211_ATTR_KEY_IDX = 8
NL80211_ATTR_KEY_CIPHER = 9
NL80211_ATTR_KEY_SEQ = 10
NL80211_ATTR_KEY_DEFAULT = 11
NL80211_ATTR_BEACON_INTERVAL = 12
NL80211_ATTR_DTIM_PERIOD = 13
NL80211_ATTR_BEACON_HEAD = 14
NL80211_ATTR_BEACON_TAIL = 15
NL80211_ATTR_STA_AID = 16
NL80211_ATTR_STA_FLAGS = 17
NL80211_ATTR_STA_LISTEN_INTERVAL = 18
NL80211_ATTR_STA_SUPPORTED_RATES = 19
NL80211_ATTR_STA_VLAN = 20
NL80211_ATTR_STA_INFO = 21
NL80211_ATTR_WIPHY_BANDS = 22
NL80211_ATTR_MNTR_FLAGS = 23
NL80211_ATTR_MESH_ID = 24
NL80211_ATTR_STA_PLINK_ACTION = 25
NL80211_ATTR_MPATH_NEXT_HOP = 26
NL80211_ATTR_MPATH_INFO = 27
NL80211_ATTR_BSS_CTS_PROT = 28
NL80211_ATTR_BSS_SHORT_PREAMBLE = 29
NL80211_ATTR_BSS_SHORT_SLOT_TIME = 30
NL80211_ATTR_HT_CAPABILITY = 31
NL80211_ATTR_SUPPORTED_IFTYPES = 32
NL80211_ATTR_REG_ALPHA2 = 33
NL80211_ATTR_REG_RULES = 34
NL80211_ATTR_MESH_CONFIG = 35
NL80211_ATTR_BSS_BASIC_RATES  = 36
NL80211_ATTR_WIPHY_TXQ_PARAMS = 37
NL80211_ATTR_WIPHY_FREQ = 38
NL80211_ATTR_WIPHY_CHANNEL_TYPE = 39
NL80211_ATTR_KEY_DEFAULT_MGMT = 40
NL80211_ATTR_MGMT_SUBTYPE = 41
NL80211_ATTR_IE = 42
NL80211_ATTR_MAX_NUM_SCAN_SSIDS = 43
NL80211_ATTR_SCAN_FREQUENCIES = 44
NL80211_ATTR_SCAN_SSIDS = 45
NL80211_ATTR_GENERATION = 46 # replaces old SCAN_GENERATION 
NL80211_ATTR_BSS = 47
NL80211_ATTR_REG_INITIATOR = 48
NL80211_ATTR_REG_TYPE = 49
NL80211_ATTR_SUPPORTED_COMMANDS = 50
NL80211_ATTR_FRAME = 51
NL80211_ATTR_SSID = 52
NL80211_ATTR_AUTH_TYPE = 53
NL80211_ATTR_REASON_CODE = 54
NL80211_ATTR_KEY_TYPE = 55
NL80211_ATTR_MAX_SCAN_IE_LEN = 56
NL80211_ATTR_CIPHER_SUITES = 57
NL80211_ATTR_FREQ_BEFORE = 58
NL80211_ATTR_FREQ_AFTER = 59
NL80211_ATTR_FREQ_FIXED = 60
NL80211_ATTR_WIPHY_RETRY_SHORT = 61
NL80211_ATTR_WIPHY_RETRY_LONG = 62
NL80211_ATTR_WIPHY_FRAG_THRESHOLD = 63
NL80211_ATTR_WIPHY_RTS_THRESHOLD = 64
NL80211_ATTR_TIMED_OUT = 65
NL80211_ATTR_USE_MFP = 66
NL80211_ATTR_STA_FLAGS2 = 67
NL80211_ATTR_CONTROL_PORT = 68
NL80211_ATTR_TESTDATA = 69
NL80211_ATTR_PRIVACY = 70 
NL80211_ATTR_DISCONNECTED_BY_AP = 71
NL80211_ATTR_STATUS_CODE = 72
NL80211_ATTR_CIPHER_SUITES_PAIRWISE = 73
NL80211_ATTR_CIPHER_SUITE_GROUP = 74
NL80211_ATTR_WPA_VERSIONS = 75
NL80211_ATTR_AKM_SUITES = 76
NL80211_ATTR_REQ_IE = 77
NL80211_ATTR_RESP_IE = 78
NL80211_ATTR_PREV_BSSID = 79
NL80211_ATTR_KEY = 80 
NL80211_ATTR_KEYS = 81
NL80211_ATTR_PID = 82
NL80211_ATTR_4ADDR = 83
NL80211_ATTR_SURVEY_INFO = 84
NL80211_ATTR_PMKID = 85
NL80211_ATTR_MAX_NUM_PMKIDS = 86
NL80211_ATTR_DURATION = 87
NL80211_ATTR_COOKIE = 88
NL80211_ATTR_WIPHY_COVERAGE_CLASS = 89
NL80211_ATTR_TX_RATES = 90
NL80211_ATTR_FRAME_MATCH = 91
NL80211_ATTR_ACK = 92
NL80211_ATTR_PS_STATE = 93
NL80211_ATTR_CQM = 94
NL80211_ATTR_LOCAL_STATE_CHANGE = 95
NL80211_ATTR_AP_ISOLATE = 96
NL80211_ATTR_WIPHY_TX_POWER_SETTING = 97
NL80211_ATTR_WIPHY_TX_POWER_LEVEL = 98
NL80211_ATTR_TX_FRAME_TYPES = 99
NL80211_ATTR_RX_FRAME_TYPES = 100
NL80211_ATTR_FRAME_TYPE = 101
NL80211_ATTR_CONTROL_PORT_ETHERTYPE = 102
NL80211_ATTR_CONTROL_PORT_NO_ENCRYPT = 103
NL80211_ATTR_SUPPORT_IBSS_RSN = 104
NL80211_ATTR_WIPHY_ANTENNA_TX = 105
NL80211_ATTR_WIPHY_ANTENNA_RX = 106
NL80211_ATTR_MCAST_RATE = 107
NL80211_ATTR_OFFCHANNEL_TX_OK = 108
NL80211_ATTR_BSS_HT_OPMODE = 109
NL80211_ATTR_KEY_DEFAULT_TYPES = 110
NL80211_ATTR_MAX_REMAIN_ON_CHANNEL_DURATION = 111
NL80211_ATTR_MESH_SETUP = 112
NL80211_ATTR_WIPHY_ANTENNA_AVAIL_TX = 113
NL80211_ATTR_WIPHY_ANTENNA_AVAIL_RX = 114
NL80211_ATTR_SUPPORT_MESH_AUTH = 115
NL80211_ATTR_STA_PLINK_STATE = 116
NL80211_ATTR_WOWLAN_TRIGGERS = 117
NL80211_ATTR_WOWLAN_TRIGGERS_SUPPORTED = 118
NL80211_ATTR_SCHED_SCAN_INTERVAL = 119
NL80211_ATTR_INTERFACE_COMBINATIONS = 120
NL80211_ATTR_SOFTWARE_IFTYPES = 121
NL80211_ATTR_REKEY_DATA = 122
NL80211_ATTR_MAX_NUM_SCHED_SCAN_SSIDS = 123
NL80211_ATTR_MAX_SCHED_SCAN_IE_LEN = 124
NL80211_ATTR_SCAN_SUPP_RATES = 125
NL80211_ATTR_HIDDEN_SSID = 126
NL80211_ATTR_IE_PROBE_RESP = 127
NL80211_ATTR_IE_ASSOC_RESP = 128
NL80211_ATTR_STA_WME = 129
NL80211_ATTR_SUPPORT_AP_UAPSD = 130
NL80211_ATTR_ROAM_SUPPORT = 131
NL80211_ATTR_SCHED_SCAN_MATCH = 132
NL80211_ATTR_MAX_MATCH_SETS = 133
NL80211_ATTR_PMKSA_CANDIDATE = 134
NL80211_ATTR_TX_NO_CCK_RATE = 135
NL80211_ATTR_TDLS_ACTION = 136
NL80211_ATTR_TDLS_DIALOG_TOKEN = 137
NL80211_ATTR_TDLS_OPERATION = 138
NL80211_ATTR_TDLS_SUPPORT = 139
NL80211_ATTR_TDLS_EXTERNAL_SETUP = 140
NL80211_ATTR_DEVICE_AP_SME = 141
NL80211_ATTR_DONT_WAIT_FOR_ACK = 142
NL80211_ATTR_FEATURE_FLAGS = 143
NL80211_ATTR_PROBE_RESP_OFFLOAD = 144
NL80211_ATTR_PROBE_RESP = 145
NL80211_ATTR_DFS_REGION = 146
NL80211_ATTR_DISABLE_HT = 147
NL80211_ATTR_HT_CAPABILITY_MASK = 148
NL80211_ATTR_NOACK_MAP = 149
NL80211_ATTR_INACTIVITY_TIMEOUT = 150
NL80211_ATTR_RX_SIGNAL_DBM = 151
NL80211_ATTR_BG_SCAN_PERIOD = 152

# bss elemets
NL80211_BSS_INVALID = 0
NL80211_BSS_BSSID = 1
NL80211_BSS_FREQUENCY = 2
NL80211_BSS_TSF = 3
NL80211_BSS_BEACON_INTERVAL = 4
NL80211_BSS_CAPABILITY = 5
NL80211_BSS_INFORMATION_ELEMENTS = 6
NL80211_BSS_SIGNAL_MBM = 7
NL80211_BSS_SIGNAL_UNSPEC = 8
NL80211_BSS_STATUS = 9
NL80211_BSS_SEEN_MS_AGO = 10
NL80211_BSS_BEACON_IE = 11

# ie lements
IE_ATTR_SSID = 0
IE_ATTR_RATES = 1
IE_ATTR_DSPARAM = 2
IE_ATTR_TIM = 5
IE_ATTR_COUNTY = 7
IE_ATTR_POWERCONSTRAINT = 32
IE_ATTR_ERP = 42
IE_ATTR_HTCAP = 45
IE_ATTR_HTOP  = 61
IE_ATTR_RSN = 48
IE_ATTR_EXTSUPRATES = 50
IE_ATTR_MESHID = 114
IE_ATTR_EXTCAP = 127

def hexprint(arr, width=8):
    from string import printable
    printable = printable.replace('\n','')
    printable = printable.replace('\r','')
    printable = printable.replace('\t','')
    printable = printable[:-10]

    ab = zip(range(0,len(arr)+width,width),range(width,len(arr)+width,width))
    for group in [arr[a:b] for a,b in ab]:
        line = '  '.join(["0x{0: <2x}".format(ord(x)) for x in group]) + "    " +\
               ''.join([g if g in printable else '.' for g in group])
        print (" "+line)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~ LibC definitions of if_nametoindex
# from http://code.activestate.com/recipes/442490-ipv6-multicast/
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

if_nametoindex = None

if if_nametoindex is None:
    try: import ctypes
    except ImportError: pass
    else:
        _libc = ctypes.CDLL("libc.so.6")
        def if_nametoindex(interfaceName):
            # if you have a better way to get the interface index, I'd love to hear
            # it...  You are supposed to be able to leave the interface number as 0,
            # but I get an exception when I try this.  (MacOS 10.4)  It may be because
            # it is a multihomed device...
            return _libc.if_nametoindex(interfaceName)

if if_nametoindex is None:
    try: import dl
    except ImportError: pass
    else:
        _libc = dl.open('libc.so')
        def if_nametoindex(interfaceName):
            # if you have a better way to get the interface index, I'd love to hear
            # it...  You are supposed to be able to leave the interface number as 0,
            # but I get an exception when I try this.  (MacOS 10.4)  It may be because
            # it is a multihomed device...
            return _libc.call('if_nametoindex', interfaceName)

if if_nametoindex is None:
    raise RuntimeError("No implementation allowing access to if_nametoindex available")

class OpenStruct:
    def __init__(self, **dic):
        self.__dict__.update(dic)
    def __getattr__(self, i):
        if i in self.__dict__:
            return self.__dict__[i]
        else:
            raise AttributeError, i
    def __setattr__(self,i,v):
        if i in self.__dict__:
            self.__dict__[i] = v
        else:
            self.__dict__.update({i:v})
        return v # i like cascates :)
    def __str__(self):
        return " ".join([str(k) + " = " + str(v) if not type(v)==list else str(k)+" = "+" \n\t ".join([str(x) for x in v]) for k,v in self.__dict__.items()])

class NLException(Exception):
    def __init__(self, num, *args):
        Exception.__init__(self,*args)
        self.num = num

    def __repr__(self):
        return "error from Netlink: %d (%s)"%(self.num,errorcode[-self.num])

    def __str__(self):
        return self.__repr__()

def ltv_parse_list(buf):
    while len(buf) > 0:
        sublen,index = struct.unpack('HH',buf[:4])
        yield buf[4:sublen]
        buf = buf[sublen:]

def ltv_parse_nested(buf):
    while len(buf) > 0:
        sublen,index = struct.unpack('HH',buf[:4])
        #hexprint(buf)
        #print "sublen,index",sublen,index

        for l,t,v in ltv_parse(buf[4:sublen]):
            yield l,t,v
            if len(v) != l-4:
                raise Exception("%d!=%d"%(len(v),l-4))

        if (sublen%4)!=0:
            sublen += (4-(sublen%4))

        buf = buf[sublen:]

def ltv_parse(buf):
    while len(buf) > 0:
        length,curtype = struct.unpack('HH',buf[:4])

        if curtype!=0:
            type = curtype

        value = buf[4:length]
        yield length,type,value
        if (length%4)!=0:
            length += (4-(length%4))
        buf = buf[length:]
        if length == 0: raise Exception("lenght==0")

def ltv_ie_parse(buf):
    while len(buf) > 1:
        type,length = struct.unpack('BB',buf[:2])
        value = buf[2:length+2]
        yield length,type,value
        if length == 0:
            pass
            #raise Exception("length (ie)==0")
        buf = buf[2+length:]

def ltv2_parse(buf, nested=[]):
   while len(buf) > 0:
       length,type = struct.unpack('HH',buf[:4])
       value = buf[4:length]

       if type in nested:
           print "nested type=%d, len=%d"%(type,length)
           subval = value
           while len(subval)>0:
               sublen,index = struct.unpack('HH',subval[:4])
               print " subnest: sublen=%d, index=%d"%(sublen,index)
               yield sublen,type,list(ltv_parse(subval[4:sublen],nested))

               if sublen==0: # sanity checks
                   raise Exception("sublen == 0")
               elif sublen != len(subval[4:sublen])+4:
                   raise Exception("sublen wrong %d!=%d"%(sublen,len(subval[4:sublen])+4))

               if (sublen%4) != 0:
                  print " subalign", 4-(sublen%4),sublen
                  sublen += 4-(sublen%4) # fix alignment

               subval = subval[sublen:]
       else:
           print "standard attr type=%d,len=%d"%(type,length)
           yield length,type,value

       if (length%4) != 0:
           print "align", 4-(length%4),length
           length += 4-(length%4) # fix alignment

       buf = buf[length:]

class NLMessage(object):
    """ A netlink message.

        Netlink message:

        |<----------------- 4 bytes ------------------->|
        |<----- 2 bytes ------>|<------- 2 bytes ------>|
        |-----------------------------------------------|
        |      Message length (including header)        |
        |-----------------------------------------------|
        |     Message type     |     Message flags      |
        |-----------------------------------------------|
        |           Message sequence number             |
        |-----------------------------------------------|
        |                 Netlink PortID                |
        |-----------------------------------------------|
        |                                               |
        .                   Payload                     .
        |_______________________________________________|

        There is usually an extra header after the the Netlink header
        (at the beginning of the payload). This extra header is specific
        of the Netlink subsystem. After this extra header, comes the
        sequence of attributes that are expressed in
        Length-Type-Value (LTV) format.
    """
    def __init__(self,type,flags,seq=None,pid=0,extra=None):
        self._pid = pid
        self._seq = randint(0,2**31) if seq is None else seq
        self._flags = flags
        self._type  = type
        self._extra = extra
        self._fmt   = struct.Struct("ihhii")

    def pack(self, fmt, *vals):
        """ Netlink Length-Type-Value (LTV) attribute:

            |<-- 2 bytes -->|<-- 2 bytes -->|<-- variable -->|
            -------------------------------------------------
            |     length    |      type     |      value     |
            -------------------------------------------------
            |<--------- header ------------>|<-- payload --->|

            The payload of the Netlink message contains sequences of
            attributes that are expressed in LTV format.
        """
        length = self._fmt.size
        if self._extra is not None: length += self._extra._fmt.size
        length += struct.calcsize(fmt)

        buf = [self._fmt.pack(length,self._type,self._flags,self._seq,self._pid)]
        if self._extra is not None: buf += self._extra.pack()
        buf += struct.pack(fmt,*vals)
        return "".join(buf)

    def unpack(self, buf):
        if hasattr(buf,"recv"):
            try:
                sock = buf
                buf = buf.recv(4096)
            except socket.error,e:
                if e.errno==11: return
                else: raise

        while len(buf) > 0:
            msglen,type,flags,seq,pid = self._fmt.unpack(buf[:self._fmt.size])
            buf = buf[self._fmt.size:]
            msglen -= self._fmt.size

            if type == NLMSG_ERROR:
                error,length,type,flags,seq,pid = struct.unpack("lLHHLL",buf[:20])
                if error==0: break
                else:        raise NLException(error)
            elif type == NLMSG_DONE:
                break

            if self._extra is not None:
                buf = buf[self._extra.unpack(buf):]
                msglen -= self._extra._fmt.size

            for l,t,v in ltv_parse(buf[:msglen]):
                yield l,t,v

            if (flags&NLM_F_MULTI):
                buf = buf[msglen:];
                if len(buf) == 0:
                    buf = sock.recv(4096)
            else:
                break;

    def __repr__(self):
        return "(type=%d, flags=%d, seq=%d, pid=%d extra=%s)"%(self._type,self._flags,self._seq,self._pid,self._extra)

class GENLMessage(object):
    def __init__(self,command,version=1):
        self._cmd = command
        self._version = version
        self._reserved = 0
        self._fmt = struct.Struct("BBH")

    def pack(self):
        return self._fmt.pack(self._cmd,self._version,self._reserved)

    def unpack(self,buf):
        self._cmd,self._version,self._reserved = self._fmt.unpack(buf[:self._fmt.size])
        return self._fmt.size

    def __repr__(self):
        return "(cmd=%d, version=%d)"%(self._cmd,self._version)

class GENLSocket(socket.socket):
    def __init__(self, family, groups=[]):
        # generate a message that ask for the supported ops
        m = NLMessage(type=GENL_ID_CTRL,
                       flags=NLM_F_REQUEST|NLM_F_ACK,
                       extra=GENLMessage(CTRL_CMD_GETFAMILY) )
        buf = m.pack('HHLHH%ds'%(len(family)+1),
                       4+4,CTRL_ATTR_FAMILY_ID,GENL_ID_CTRL,
                       4+len(family)+1,CTRL_ATTR_FAMILY_NAME,family)

        # last id is bus id  see NETLINK_* in linux/netlink.h, 16 is generic
        socket.socket.__init__(self,socket.AF_NETLINK,socket.SOCK_RAW,16)
        # PID=0 means AUTOPID, groups = 0
        self.bind((0,0))

        # send the resolver msg
        self.send(buf)

        # and unpack the rx'ed one
        self._ops,self._groups = [],{}
        for l,t,v in m.unpack(self.recv(4096)):
            if t==CTRL_ATTR_FAMILY_ID:
                self._id = struct.unpack("H",v)[0]

            elif t==CTRL_ATTR_OPS:
                for l,t,v in ltv_parse_nested(v):
                    if t==CTRL_ATTR_OP_ID:
                        self._ops += struct.unpack('L',v)

            elif t==CTRL_ATTR_MCAST_GROUPS:
                key,value=None,None

                for l,t,v in ltv_parse_nested(v):
                    if t==CTRL_ATTR_MCAST_GRP_NAME: key = v[:v.index('\0')]
                    elif t==CTRL_ATTR_MCAST_GRP_ID: value = struct.unpack('L',v)[0]

                    if key is not None and value is not None:
                        self._groups[key] = value
                        key,value = None,None

        #self.setblocking(0)
        for group in groups:
            self.setsockopt(SOL_NETLINK,NETLINK_ADD_MEMBERSHIP,self._groups[group])

        self._msg = NLMessage(type=self._id,
                       flags=NLM_F_REQUEST|NLM_F_ACK,
                       extra=GENLMessage(CTRL_CMD_GETFAMILY) )

    def rx(self):
        try:
            return self._msg.unpack(self)
        except socket.error,e:
            if e.errno==11: return []
            else: raise

    def tx(self, cmd, args=[], flags=NLM_F_REQUEST|NLM_F_ACK):
        msg = NLMessage(self._id, flags, extra=GENLMessage(cmd))

        def build_ltv(ltv):
            fmt,val="",[]
            for l,t,v in ltv:
                val.extend([l+4,t,v])
                fmt += "HH"

                if (type(v)==tuple or type(v)==list) and l==0:
                    rf,rv = build_ltv([v])
                    val.pop() # remove last value and add length
                    val[-2] += struct.calcsize(rf)
                    fmt += rf
                    val += rv
                elif type(v)==tuple or type(v)==list:
                    if l==8: rf = "HHQ"*len(v)
                    elif l==4: rf = "HHL"*len(v)
                    elif l==2: rf = "HHH"*len(v)
                    elif l==1: rf = "HHB"*len(v)
                    else: raise Exception()
                    val.pop()
                    val[-2] = struct.calcsize(rf)+4
                    fmt += rf
                    for i,v in zip(range(len(v)),v):
                        val.extend((l+4,i+1,v))
                elif type(v) == str: fmt += "%ds"%l
                elif l==8: fmt += "Q"
                elif l==4: fmt += "L"
                elif l==2: fmt += "H"
                elif l==1: fmt += "B"
                else:
                    raise Exception("don't know what todo with value")
            return fmt,val

        fmt,val=build_ltv(args)

        buf = msg.pack(fmt,*val)
        #print "-> ", len(buf), " ".join([hex(ord(x)) for x in buf])
        return self.send(buf)

class NL80211(GENLSocket):
    def __init__(self, device="wlan0"):
        GENLSocket.__init__(self,"nl80211", ["scan", "mlme"])
        self._if_index = if_nametoindex(device)

        if self._if_index == 0:
            raise Exception("unable to find interface '%s'"%device)

    def trigger_scan(self,freqs=None):
        if freqs is None:
            self.tx(NL80211_CMD_TRIGGER_SCAN,
                [(4,NL80211_ATTR_IFINDEX, self._if_index),
                 (0,NL80211_ATTR_SCAN_SSIDS, (0,1,""))] )
        else:
            self.tx(NL80211_CMD_TRIGGER_SCAN,
                [(4,NL80211_ATTR_IFINDEX, self._if_index),
                 (0,NL80211_ATTR_SCAN_SSIDS, (0,1,"")),
                 (4,NL80211_ATTR_SCAN_FREQUENCIES,freqs)
                ])

    def sched_scan_start(self, period):
        self.tx(NL80211_CMD_START_SCHED_SCAN,
            [(4,NL80211_ATTR_IFINDEX, self._if_index),
             (0,NL80211_ATTR_SCAN_SSIDS, (0,1,"")),
             (4,NL80211_ATTR_SCHED_SCAN_INTERVAL, period)])

    def sched_scan_stop(self):
        self.tx(NL80211_CMD_STOP_SCHED_SCAN,
            [(4,NL80211_ATTR_IFINDEX, self._if_index)])

    def get_scan_results(self):
        self.tx(NL80211_CMD_GET_SCAN,
            [(4,NL80211_ATTR_IFINDEX, self._if_index)],
            flags=NLM_F_REQUEST|NLM_F_ACK|NLM_F_DUMP)

    def handle_event(self):
        msg = OpenStruct(type="none")

        for l,t,v in self.rx():
            if self._msg._extra._cmd == NL80211_CMD_NEW_SCAN_RESULTS:
                msg.type = "scan"
                if t==NL80211_ATTR_SCAN_SSIDS:
                    msg.ssids = ":".join([hex(ord(x)) for x in v])
                elif t==NL80211_ATTR_SCAN_FREQUENCIES:
                    msg.freqs = [struct.unpack("L",v[a:a+4])[0] for a in range(0,len(v),4)]
                elif t==NL80211_ATTR_GENERATION:
                    msg.gen = struct.unpack("L", v)[0]
                elif t==NL80211_ATTR_IFINDEX:
                    msg.ifindex = struct.unpack("L",v)[0]
                elif t==NL80211_ATTR_BSS:
                    bss = OpenStruct()
                    for l,t,v in ltv_parse(v):
                        if t==NL80211_BSS_BSSID:
                            bss.bssid = v
                        elif t==NL80211_BSS_SIGNAL_MBM:
                            bss.signal = struct.unpack('l',v)[0]
                        elif t==NL80211_BSS_FREQUENCY:
                            bss.frequency = struct.unpack('L',v)[0]
                        elif t==NL80211_BSS_INFORMATION_ELEMENTS or\
                             t==NL80211_BSS_BEACON_IE:
                            ie = OpenStruct()
                            for l,t,v in ltv_ie_parse(v):
                                if t==IE_ATTR_SSID: ie.ssid = v
                            if not hasattr(bss,"ie"): bss.ie = [ie]
                            else: bss.ie.append(ie)
                        else:
                            pass
                    if not hasattr(msg,"bss"): msg.bss = [bss]
                    else: msg.bss.append(bss)
                else:
                    pass

            elif self._msg._extra._cmd == NL80211_CMD_TRIGGER_SCAN:
                msg.type = "trigger"
                if t==NL80211_ATTR_SCAN_FREQUENCIES:
                    msg.freqs = [struct.unpack("L",x)[0] for x in ltv_parse_list(v)]
                elif t==NL80211_ATTR_SCAN_SSIDS:
                    pass

            else:
                print "unknown attr type=%d len=%d %s"%(t,l,v)
                pass
                #self.get_scan_results()

        return msg

    def ops(self):
        return [ k for k,v in globals().items() if v in self._ops and k.startswith("NL80211_CMD") ]

if __name__ == "__main__":
    n = NL80211("wlan1")
    n.trigger_scan()
    live_freqs, supported_freqs = None,None

    atexit.register(n.sched_scan_stop)

    # now select until done
    rlist = []
    while not stdin in rlist:
        if len(rlist) == 0:
            #n.get_scan_results()
            pass # timeout

        elif n in rlist:
            try:
                msg = n.handle_event()
                if msg.type=="scan" and not hasattr(msg,"bss"):
                    n.get_scan_results()
                    if supported_freqs is None:
                        supported_freqs = set(msg.freqs[1::2])
                        n.trigger_scan()
                    elif live_freqs is None:
                        n.trigger_scan()
                    else:
                        dead_freqs = supported_freqs - live_freqs
                        k = int(ceil(len(dead_freqs)*.3))
                        toscan = list(live_freqs) + sample(dead_freqs,2)
                        n.trigger_scan(toscan)
                        print len(live_freqs), k, len(toscan), toscan

                if hasattr(msg, "bss"):
                    for bss in msg.bss:
                        bssid = ":".join([hex(ord(x))[2:] for x in bss.bssid])
                        print "(%s@%dMhz)\t%2.2fdBm\t%s"%(bssid,bss.frequency,bss.signal/100.0,bss.ie[0].ssid)
                    live_freqs = set([bss.frequency for bss in msg.bss])
            except NLException,e:
                if e.num == -100:
                    print "ERROR: network is down!"
                    exit(-1)
                elif e.num == -1:
                    print "ERROR: scanning not allowed!"
                    exit(-1)
                else:
                    from traceback import print_exception,print_exc
                    print_exc()

        rlist,wlist,xlist=select([n,stdin],[],[],0.5)

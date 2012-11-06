import sys
import fcntl

def all_interfaces():
    f  = open("/proc/net/dev",'r')
    il = f.read().split('\n')
    f.close()

    # remove headers
    il.pop(0); il.pop(0)

    return [line.replace(' ','').split(':')[0] for line in il if len(line)>0]

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


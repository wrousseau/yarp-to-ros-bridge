import os, sys, socket
import rospkg, rosgraph, genmsg

class ROSMsgException(Exception): pass

try:
    from cStringIO import StringIO  # Python 2.x
except ImportError:
    from io import StringIO  # Python 3.x

def getString(topic):
    try:
        val = rosgraph.Master('/rostopic').getTopicTypes()
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    # exact match first, followed by prefix match
    matches = [(t, t_type) for t, t_type in val if t == topic]
    if not matches:
        matches = [(t, t_type) for t, t_type in val if topic.startswith(t+'/')]
        # choose longest match
        matches.sort(key=itemgetter(0), reverse=True)
    if matches:
        msg = matches[0][1]
    rospack = rospkg.RosPack()
    search_path = {}
    for p in rospack.list():
        search_path[p] = [os.path.join(rospack.get_path(p), 'msg')]
    context = genmsg.MsgContext.create_default()
    try:
        spec = genmsg.load_msg_by_type(context, msg, search_path)
        genmsg.load_depends(context, spec, search_path)
    except Exception as e:
        raise ROSMsgException("Unable to load msg [%s]: %s"%(msg, e))
    str = spec_to_str(context,spec)
    str = str.replace('\n',' ')
    str = ' '.join(str.split())
    return str

def spec_to_str(msg_context, spec, buff=None, indent=''):
    """
    Convert spec into a string representation. Helper routine for MsgSpec.
    :param indent: internal use only, ``str``
    :param buff: internal use only, ``StringIO``
    :returns: string representation of spec, ``str``
    """
    ## time as msg spec. time is unsigned 
    TIME_MSG     = "uint32 secs uint32 nsecs"
    ## duration as msg spec. duration is just like time except signed
    DURATION_MSG = "int32 secs int32 nsecs"

    if buff is None:
        buff = StringIO()
    for type_, name in zip(spec.types, spec.names):
        base_type = genmsg.msgs.bare_msg_type(type_)
        if base_type == "time":
            buff.write("%s%s"%(indent,TIME_MSG))
        elif base_type == "duration":
            buff.write("%s%s"%(indent,DURATION_MSG))
        elif base_type in genmsg.msgs.BUILTIN_TYPES:
            buff.write("%s%s %s\n"%(indent, type_, name))
        else:
            subspec = msg_context.get_registered(base_type)
            spec_to_str(msg_context, subspec, buff, indent + '  ')
    return buff.getvalue()
class Param(object):
    mapping = {}

    def __init__(self, node):
        if node.tag != 'param':
            raise ValueError('expected <param>, got <%s>' % node.tag)
        self.name = node.attrib['name']
        self.dtype = node.attrib['type']
        self.default = node.attrib.get('default', None)
        self.skip = node.attrib.get('skip', 'false').lower() in ('true', 'yes', '1')
        self.write_in = True
        self.write_out = True

    def mandatory(self):
        return self.default is None

    def optional(self):
        return self.default is not None

    def ctype(self):
        return self.dtype

    def ctype_normalized(self):
        return self.ctype().replace('::', '__')

    def htype(self):
        return self.dtype

    def cdefault(self):
        return self.default

    def hdefault(self):
        return self.default

    @staticmethod
    def register_type(dtype, clazz):
        Param.mapping[dtype] = clazz

    @staticmethod
    def factory(node):
        dtype = node.attrib['type']
        return Param.mapping[dtype](node)

class ParamInt(Param):
    def __init__(self, node):
        super(ParamInt, self).__init__(node)

    def htype(self):
        return 'number'

class ParamFloat(Param):
    def __init__(self, node):
        super(ParamFloat, self).__init__(node)

    def htype(self):
        return 'number'

class ParamDouble(Param):
    def __init__(self, node):
        super(ParamDouble, self).__init__(node)

    def htype(self):
        return 'number'

class ParamString(Param):
    def __init__(self, node):
        super(ParamString, self).__init__(node)

    def cdefault(self):
        if self.default is None: return None
        return '"%s"' % self.default.replace('\\','\\\\').replace('"','\\"')

    def ctype(self):
        return 'std::string'

    def hdefault(self):
        if self.default is None: return None
        return self.cdefault().replace('\\','\\\\').replace('"','\\"')

class ParamBool(Param):
    def __init__(self, node):
        super(ParamBool, self).__init__(node)

class ParamTable(Param):
    def __init__(self, node):
        super(ParamTable, self).__init__(node)
        self.itype = node.attrib.get('item-type', None)
        self.minsize = int(node.attrib.get('minsize', 0))
        self.maxsize = int(node.attrib['maxsize']) if 'maxsize' in node.attrib else None
        if 'size' in node.attrib:
            self.minsize = int(node.attrib['size'])
            self.maxsize = int(node.attrib['size'])
        if self.itype is None:
            self.write_in = False
            self.write_out = False

    def item_dummy(self):
        n = type('dummyNode', (object,), dict(tag='param', attrib={'name': 'dummy', 'type': self.itype}))
        return Param.factory(n)

    def ctype(self):
        if self.itype is not None:
            return 'std::vector<%s>' % self.item_dummy().ctype()
        else:
            return 'void *'

    def ctype_normalized(self):
        return self.item_dummy().ctype().replace('::', '__')

    def htype(self):
        return 'table' + ('_%d' % self.minsize if self.minsize else '')

    def cdefault(self):
        if self.default is not None:
            d = self.default
            d = 'boost::assign::list_of' + ''.join(map(lambda x: '(%s)' % x.strip(), d.strip()[1:-1].split(',')))
            return d

class ParamStruct(Param):
    def __init__(self, node, name):
        super(ParamStruct, self).__init__(node)
        self.structname = name
        self.xoptional = False
        if self.default is not None:
            if self.default == '{}':
                self.xoptional = True
            else:
                raise ValueError('default value not supported in <struct>')

    def mandatory(self):
        return not self.xoptional

    def optional(self):
        return self.xoptional

    def cdefault(self):
        return None

Param.register_type('int', ParamInt)
Param.register_type('float', ParamFloat)
Param.register_type('double', ParamDouble)
Param.register_type('string', ParamString)
Param.register_type('bool', ParamBool)
Param.register_type('table', ParamTable)

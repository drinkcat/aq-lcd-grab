from collections import defaultdict
from skidl import Pin, Part, Alias, SchLib, SKIDL, TEMPLATE

from skidl.pin import pin_types

SKIDL_lib_version = '0.0.1'

aq_lcd_grab = SchLib(tool=SKIDL).add_parts(*[
        Part(**{ 'name':'Conn_01x39_Socket', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'Conn_01x39_Socket'}), 'ref_prefix':'J', 'fplist':[''], 'footprint':'FH26W:FH26W39S03SHW60', 'keywords':'connector', 'description':'Generic connector, single row, 01x39, script generated', 'datasheet':'', 'pins':[
            Pin(num='1',name='Pin_1',func=pin_types.PASSIVE,unit=1),
            Pin(num='2',name='Pin_2',func=pin_types.PASSIVE,unit=1),
            Pin(num='3',name='Pin_3',func=pin_types.PASSIVE,unit=1),
            Pin(num='4',name='Pin_4',func=pin_types.PASSIVE,unit=1),
            Pin(num='5',name='Pin_5',func=pin_types.PASSIVE,unit=1),
            Pin(num='6',name='Pin_6',func=pin_types.PASSIVE,unit=1),
            Pin(num='7',name='Pin_7',func=pin_types.PASSIVE,unit=1),
            Pin(num='8',name='Pin_8',func=pin_types.PASSIVE,unit=1),
            Pin(num='9',name='Pin_9',func=pin_types.PASSIVE,unit=1),
            Pin(num='10',name='Pin_10',func=pin_types.PASSIVE,unit=1),
            Pin(num='11',name='Pin_11',func=pin_types.PASSIVE,unit=1),
            Pin(num='12',name='Pin_12',func=pin_types.PASSIVE,unit=1),
            Pin(num='13',name='Pin_13',func=pin_types.PASSIVE,unit=1),
            Pin(num='14',name='Pin_14',func=pin_types.PASSIVE,unit=1),
            Pin(num='15',name='Pin_15',func=pin_types.PASSIVE,unit=1),
            Pin(num='16',name='Pin_16',func=pin_types.PASSIVE,unit=1),
            Pin(num='17',name='Pin_17',func=pin_types.PASSIVE,unit=1),
            Pin(num='18',name='Pin_18',func=pin_types.PASSIVE,unit=1),
            Pin(num='19',name='Pin_19',func=pin_types.PASSIVE,unit=1),
            Pin(num='20',name='Pin_20',func=pin_types.PASSIVE,unit=1),
            Pin(num='21',name='Pin_21',func=pin_types.PASSIVE,unit=1),
            Pin(num='22',name='Pin_22',func=pin_types.PASSIVE,unit=1),
            Pin(num='23',name='Pin_23',func=pin_types.PASSIVE,unit=1),
            Pin(num='24',name='Pin_24',func=pin_types.PASSIVE,unit=1),
            Pin(num='25',name='Pin_25',func=pin_types.PASSIVE,unit=1),
            Pin(num='26',name='Pin_26',func=pin_types.PASSIVE,unit=1),
            Pin(num='27',name='Pin_27',func=pin_types.PASSIVE,unit=1),
            Pin(num='28',name='Pin_28',func=pin_types.PASSIVE,unit=1),
            Pin(num='29',name='Pin_29',func=pin_types.PASSIVE,unit=1),
            Pin(num='30',name='Pin_30',func=pin_types.PASSIVE,unit=1),
            Pin(num='31',name='Pin_31',func=pin_types.PASSIVE,unit=1),
            Pin(num='32',name='Pin_32',func=pin_types.PASSIVE,unit=1),
            Pin(num='33',name='Pin_33',func=pin_types.PASSIVE,unit=1),
            Pin(num='34',name='Pin_34',func=pin_types.PASSIVE,unit=1),
            Pin(num='35',name='Pin_35',func=pin_types.PASSIVE,unit=1),
            Pin(num='36',name='Pin_36',func=pin_types.PASSIVE,unit=1),
            Pin(num='37',name='Pin_37',func=pin_types.PASSIVE,unit=1),
            Pin(num='38',name='Pin_38',func=pin_types.PASSIVE,unit=1),
            Pin(num='39',name='Pin_39',func=pin_types.PASSIVE,unit=1)], 'unit_defs':[] })])
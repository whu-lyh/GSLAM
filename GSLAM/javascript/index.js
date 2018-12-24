const nbind = require('nbind')
module.exports = nbind.init(__dirname).lib



obj={b:false,i:1,d:2.3,s:"hello",a:[1,2,"4"],o:{"v":8}}

msg=module.exports.Messenger()
pub=msg.advertise("Object","obj",0,false)

sub=msg.subscribe("Object","obj",console.log);

pub.publish(54.)
pub.publish(false)
pub.publish("str")
pub.publish([1,3])
pub.publish([1,"3"])
//pub.publish(obj)

pubStrMap=msg.advertise("StrMap","obj",0,false)
subStrMap=msg.subscribe("StrMap","obj",console.log);

pubStrMap.publish({"first":"1","second":"2"})

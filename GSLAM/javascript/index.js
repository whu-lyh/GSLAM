const nbind = require('nbind')
module.exports = nbind.init(__dirname).lib

function callback(obj){
  console.log("received ",obj)
}

obj={b:false,i:1,d:2.3,s:"hello",a:[1,2,"4"],o:{"v":8}}

gslam=module.exports
msg=module.exports.Messenger()

pub=msg.advertise(gslam.MapFrame,"obj",0,false)
sub=msg.subscribe(gslam.MapFrame,"obj",callback);

//dataset=gslam.Dataset("/data/zhaoyong/Dataset/mav0/.euroc")
//fr=dataset.grabFrame()
//console.log("received ",fr)
//pub.publish(fr)


pub=msg.advertise("Object","Object",0,false)
sub=msg.subscribe("Object","Object",callback);
pub.publish(0)
pub.publish(false)
pub.publish("str")
pub.publish([1,3])
pub.publish([1,"3"])
pub.publish(obj)


parameters={slam:{},mapfusion:{}}
parameters.slam.thread=true
parameters.mapfusion.mode="Multiband"
//parameters.slam.frame=[fr]
pub.publish(parameters)

svar=gslam.Svar.instance()

var fs=require('fs');
data=JSON.parse(fs.readFileSync('/usr/share/unity/client-scopes.json')) 
callback(data)

svar.loadJson(data)
svar.loadJson(obj)
svar.dumpAllVars()


//pubStrMap=msg.advertise("StrMap","obj",0,false)
//subStrMap=msg.subscribe("StrMap","obj",callback);

//pubStrMap.publish({"first":"1","second":"2"})

//setInterval(()=>{},50)

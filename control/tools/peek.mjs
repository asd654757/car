// 读一下控制环当前遥测（只发 HTTP，不碰串口），跑完即删。
const s = (await (await fetch('http://127.0.0.1:8765/state')).json()).snapshot;
const spd = s.wheels.map((w) => w.driveSpd).join(',');
const wire = s.onWire.join(',');
const cmd = s.wheels.map((w) => w.driveCmd).join(',');
console.log(`${s.state} ${s.reason || ''} | 线上[${wire}] 自报命令[${cmd}] 实际转速[${spd}] | 写${s.eeprom.writes} 延后${s.eeprom.deferred} 停${s.eeprom.stops} 失败${s.eeprom.failed} | 拍${s.ticks}`);

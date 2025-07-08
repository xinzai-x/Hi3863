import { SearchView } from "../component/SearchView";
import BHView from "../deviceview/BHView";
import BuzzerView from "../deviceview/BuzzerView";
import DC_MotorView from "../deviceview/DC_MotorView";
import DHTView from "../deviceview/DHTView";
import LEDView from "../deviceview/LEDView";
import DeviceLinkInfoView from "../pages/DeviceLinkInfoView";
import MQTTView from "../utils/MQTTView";

//跳转页声明
@Builder
function dhtInfoView() {
  DHTView()
}

@Builder
function mqttInfoView() {
  MQTTView()
}

@Builder
function ledInfoView() {
  LEDView()
}

@Builder
function SearchViewBuilder() {
  NavDestination() {
    SearchView()
  }
  .hideTitleBar(true)
}

@Builder
function DeviceLinkInfoViewBuilder() {
  NavDestination() {
    DeviceLinkInfoView()
  }.title('设备连接信息')
}

@Builder
function BHViewViewBuilder() {
  BHView() //光照传感器
}

@Builder
function BuzzerViewBuilder() {
  BuzzerView() //蜂鸣器
}

@Builder
function DC_MotorViewBuilder() {
  DC_MotorView() //直流电机
}

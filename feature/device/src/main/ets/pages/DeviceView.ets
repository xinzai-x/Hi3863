import {
  deleteDeviceApi,
  deviceInfoInterface,
  DeviceStateManager,
  InfoParams,
  getIconByType,
  MKNavPathStack,
  PreferenceManagerFactory,
} from "basic";
import deviceSubcomponent from "../component/DeviceSubcomponent";
import PopInit from "basic/src/main/ets/utils/StateStoreUtil";
import SheetBuilder from "../BuilderFunction/SheetFunction";
import { promptAction } from "@kit.ArkUI";

const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo')

@Entry
@Component
struct DeviceView {
  @State isShowSheet: boolean = false
  @State isPresent: boolean = false
  @State selectButton: number = 0
  //控制半模态弹窗
  @StorageLink('popInfo')
  popUIState: Record<string, boolean> = {}
  //是否登陆
  @StorageLink('isLoggedIn') isLoggedIn: boolean = true;

  async RefreshDevice() {
    await DeviceStateManager.loadDeviceList()
  }

  aboutToAppear(): void {
    if (userPref.getValue('token') !== '') {
      AppStorage.setOrCreate('isLoggedIn', true); //启动时有token置登陆状态为开
      this.RefreshDevice()
    }
  }

  //菜单栏增加设备信息
  TooTmp: NavigationMenuItem = {
    value: "增加设备",
    icon: "resources/base/media/ic_public_add_filled.svg",
    action: () => PopInit()
  }
  TooTmp1: NavigationMenuItem = {
    value: "搜索",
    icon: "resources/base/media/ic_public_search.svg",
    action: () => {
      MKNavPathStack.pushPathByName('SearchView', null)
    }
  }

  build() {
    Navigation() {
      Column() {
        Scroll() {
          Column() {
            if (this.isLoggedIn) {
              GridRow({
                columns: {
                  xs: 1,
                  sm: 2,
                  md: 4,
                  lg: 6,
                  xl: 8,
                  xxl: 10
                },
                gutter: 12
              }) {
                LazyForEach(DeviceStateManager.getDataSource(), (item: deviceInfoInterface) => {
                  GridCol() {
                    deviceSubcomponent({
                      title: item.deviceName,
                      location: item.location,
                      linkMode: item.pipeline,
                      icon: getIconByType(item.deviceType),
                    })
                  }
                  .gesture( //组合手势
                    GestureGroup(GestureMode.Exclusive,
                      TapGesture({ count: 1 })//单击
                        .onAction(async () => {
                          if (item.deviceType === '温湿度传感器') {
                            if (item.pipeline === 'MQTT') {
                              if (userPref.getValue('mqtt_Flag') === 'false') {
                                promptAction.openToast({ message: '请先配置MQTT' })
                                MKNavPathStack.pushPathByName('MQTTInfoView', null)
                              } else {
                                //传入子页面的参数
                                const params: InfoParams = {
                                  deviceId: item.deviceViewId,
                                  deviceName: item.deviceName,
                                  icon: getIconByType(item.deviceType),
                                  linkMode: item.pipeline,
                                };
                                console.log(JSON.stringify(params))
                                MKNavPathStack.pushPathByName('DHTInfoView', params)
                              }
                            } else if (item.pipeline === 'HTTP') {
                              promptAction.openToast({ message: '暂不支持' })
                            }
                          } else if (item.deviceType === 'LED灯') {
                            if (item.pipeline === 'MQTT') {
                              if (userPref.getValue('mqtt_Flag') === 'false') {
                                promptAction.openToast({ message: '请先配置MQTT' })
                                MKNavPathStack.pushPathByName('MQTTInfoView', null)
                              } else {
                                //传入子页面的参数
                                const params: InfoParams = {
                                  deviceId: item.deviceViewId,
                                  deviceName: item.deviceName,
                                  icon: getIconByType(item.deviceType),
                                  linkMode: item.pipeline,
                                };
                                console.log(JSON.stringify(params))
                                MKNavPathStack.pushPathByName('LEDInfoView', params)
                              }
                            } else if (item.pipeline === 'HTTP') {
                              promptAction.openToast({ message: '暂不支持' })
                            }
                          } else if (item.deviceType === '光照传感器') {
                            if (item.pipeline === 'MQTT') {
                              if (userPref.getValue('mqtt_Flag') === 'false') {
                                promptAction.openToast({ message: '请先配置MQTT' })
                                MKNavPathStack.pushPathByName('MQTTInfoView', null)
                              } else {
                                //传入子页面的参数
                                const params: InfoParams = {
                                  deviceId: item.deviceViewId,
                                  deviceName: item.deviceName,
                                  icon: getIconByType(item.deviceType),
                                  linkMode: item.pipeline,
                                };
                                console.log(JSON.stringify(params))
                                MKNavPathStack.pushPathByName('BHView', params)
                              }
                            } else if (item.pipeline === 'HTTP') {
                              promptAction.openToast({ message: '暂不支持' })
                            }
                          } else if (item.deviceType === '蜂鸣器') {
                            if (item.pipeline === 'MQTT') {
                              if (userPref.getValue('mqtt_Flag') === 'false') {
                                promptAction.openToast({ message: '请先配置MQTT' })
                                MKNavPathStack.pushPathByName('MQTTInfoView', null)
                              } else {
                                //传入子页面的参数
                                const params: InfoParams = {
                                  deviceId: item.deviceViewId,
                                  deviceName: item.deviceName,
                                  icon: getIconByType(item.deviceType),
                                  linkMode: item.pipeline,
                                };
                                console.log(JSON.stringify(params))
                                MKNavPathStack.pushPathByName('BuzzerView', params)
                              }
                            } else if (item.pipeline === 'HTTP') {
                              promptAction.openToast({ message: '暂不支持' })
                            }
                          } else if (item.deviceType === '直流电机') {
                            if (item.pipeline === 'MQTT') {
                              if (userPref.getValue('mqtt_Flag') === 'false') {
                                promptAction.openToast({ message: '请先配置MQTT' })
                                MKNavPathStack.pushPathByName('MQTTInfoView', null)
                              } else {
                                //传入子页面的参数
                                const params: InfoParams = {
                                  deviceId: item.deviceViewId,
                                  deviceName: item.deviceName,
                                  icon: getIconByType(item.deviceType),
                                  linkMode: item.pipeline,
                                };
                                console.log(JSON.stringify(params))
                                MKNavPathStack.pushPathByName('DC_MotorView', params)
                              }
                            } else if (item.pipeline === 'HTTP') {
                              promptAction.openToast({ message: '暂不支持' })
                            }
                          }
                        }),
                      LongPressGesture()//长按
                        .onAction(() => {
                          this.getUIContext().showAlertDialog({
                            title: `删除${item.deviceName}`,
                            message: "是否确定删除该设备,此操作无法撤销", // 要显示的提示内容
                            buttons: [
                              {
                                value: '取消',
                                fontColor: $r('app.color.black'),
                                action: () => {
                                  console.log('点击了取消')
                                }
                              },
                              {
                                value: '确定',
                                fontColor: $r('app.color.red'),
                                action: async () => {
                                  const res = await deleteDeviceApi(item.deviceViewId)
                                  if (res.data.code === 200) {
                                    setTimeout(async () => {
                                      await DeviceStateManager.loadDeviceList() //添加设备后刷新数据触发UI更新
                                      promptAction.openToast({ message: '删除成功' })
                                    }, 500)
                                  } else {
                                    promptAction.openToast({ message: '删除失败' })
                                  }
                                  console.log('点击了确定');
                                }
                              }
                            ]
                          })
                          console.log('长按', item.deviceViewId.toString());
                        })
                    ))
                }, (item: deviceInfoInterface) => item.deviceName)
              }
            } else {
              Column() {
                Text('暂无设备').fontColor($r('app.color.common_gray_01'))
              }.justifyContent(FlexAlign.Center).height('100%')
            }
          }
          .width('100%')
          .align(Alignment.TopStart)
          .padding(16)
        }
        .scrollBar(BarState.Off)
      }.backgroundColor($r('app.color.common_gray_bg'))
      .width('100%')
      .height('100%')
    }
    .mode(NavigationMode.Stack) //强制让 Navigation单栏布局
    .title('我的设备')
    .hideToolBar(true)
    .menus([this.TooTmp1, this.TooTmp])
    .bindSheet(this.popUIState.isShowSheet, SheetBuilder(), {
      detents: [SheetSize.MEDIUM, SheetSize.LARGE],
      preferType: SheetType.BOTTOM,
      showClose: false,
      title: { title: '添加设备' },
    })
  }
}

export default DeviceView

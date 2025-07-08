import { promptAction } from "@kit.ArkUI"
import { deviceInfoInterface, DeviceStateManager, postDeviceApi, PreferenceManagerFactory } from "basic"
import { JSON } from "@kit.ArkTS"

const type: SelectOption[] = [
  { value: 'LED灯', icon: 'https://i.postimg.cc/DwmPSdjz/led.png' },
  { value: '温湿度传感器', icon: 'https://i.postimg.cc/PJtmTmvT/humiture.png' },
  { value: '光照传感器', icon: 'https://i.postimg.cc/MpKN12Lj/BH.png' },
  { value: '蜂鸣器', icon: 'https://i.postimg.cc/xjg9rVJs/buzzer.png' },
  { value: '直流电机', icon: 'https://i.postimg.cc/BbbPxv2V/ele-mach.png' },
]
const protocol: SelectOption[] = [
  { value: 'MQTT', icon: 'https://i.postimg.cc/V6KzP6wR/MQTT.png' },
  { value: 'HTTP', icon: 'https://i.postimg.cc/rpnqvsM9/HTTP.png' }
]
const devicePosition: SelectOption[] = [
  { value: '卧室', icon: 'resources/base/media/bedchamber.svg' },
  { value: '客厅', icon: 'resources/base/media/living.svg' },
  { value: '卫生间', icon: 'resources/base/media/toilet.svg' },
]
let deviceInfo: deviceInfoInterface = {
  deviceViewId: '',
  deviceName: '',
  deviceType: '',
  pipeline: '',
  location: '',
  linkstatus: ''
}


interface api_device_info {
  name: string,
  type: string,
  location: string,
  protocol: string
}

@Builder
function SheetBuilder() {

  Column({ space: 16 }) {
    Row() {
      Text('设备名称：').fontWeight(FontWeight.Medium)
      TextInput({ placeholder: '设备名不可重复' }).layoutWeight(1)
        .onChange((value) => {
          deviceInfo.deviceName = value
          console.log('名称：', deviceInfo.deviceName)
        })
    }.width('100%')

    Row() {
      Text('设备类型：').fontWeight(FontWeight.Medium)
      Select(type)
        .layoutWeight(1)
        .optionWidth(200)
        .value('请选择')
        .menuAlign(MenuAlignType.CENTER)
        .optionHeight(200)
        .onSelect((index: number) => {
          deviceInfo.deviceType = type[index].value.toString()
          console.log('设备类型', deviceInfo.deviceType)
        })
    }.width('100%')

    Row() {
      Text('连接协议：').fontWeight(FontWeight.Medium)
      Select(protocol)
        .layoutWeight(1)
        .optionWidth(200)
        .value('请选择')
        .menuAlign(MenuAlignType.CENTER)
        .optionHeight(200)
        .onSelect((index: number) => {
          deviceInfo.pipeline = protocol[index].value.toString()
          console.log('连接协议', deviceInfo.pipeline)
        })
    }.width('100%')

    Row() {
      Text('设备位置：').fontWeight(FontWeight.Medium)
      Select(devicePosition)
        .layoutWeight(1)
        .optionWidth(200)
        .value('请选择')
        .optionHeight(200)
        .menuAlign(MenuAlignType.CENTER)
        .onSelect((index: number) => {
          deviceInfo.location = devicePosition[index].value.toString()
          console.log('设备位置', deviceInfo.location)
        })
    }.width('100%')

    selectButton() //按钮组件
  }.padding(16).width('100%').height('100%')
  .justifyContent(FlexAlign.Center)
}

@Component
struct selectButton {
  @StorageLink('popInfo')
  popUIState: Record<string, boolean> = {}

  build() {
    Column() {
      Button('创建设备').width('90%').margin({ top: 30 })
        .onClick(async () => {
          const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo')
          if (userPref.getValue('token') !== '') {
            console.log('数据：', deviceInfo.deviceName)
            //保存填写信息，后存入API
            const valueBucket: api_device_info = {
              name: deviceInfo.deviceName,
              type: deviceInfo.deviceType,
              location: deviceInfo.location,
              protocol: deviceInfo.pipeline
            }
            // 判断传入的信息是否有空？
            const hasEmptyProperties = (obj: api_device_info): boolean => {
              return (
                obj.name === '' || obj.type === '' || obj.location === '' || obj.protocol === ''// 空字符串
              );
            };
            if (hasEmptyProperties(valueBucket) === false) {
              try {
                const res = await postDeviceApi(valueBucket)
                if (res.data.code === 201) {
                  console.log('设备信息：', JSON.stringify(res))
                  this.popUIState.isShowSheet = false //关闭半模态弹窗
                  await DeviceStateManager.loadDeviceList() //添加设备后刷新数据触发UI更新
                  promptAction.openToast({ message: '创建成功' })
                }
              } catch (error) {
                switch (error.response.status) {
                  case 400:
                    promptAction.openToast({ message: '参数无效或设备名已存在' })
                    break;
                  case 401:
                    promptAction.openToast({ message: '需有效token' })
                    break;
                  case 500:
                    promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                    break;
                  default:
                    promptAction.openToast({ message: '保存失败，请稍后重试' })
                    break;
                }
              }
            } else {
              promptAction.openToast({ message: '请填写完整信息!' })
            }
          } else {
            promptAction.openToast({ message: '请登录后再创建设备' })
          }
        })
    }.width('90%')
  }
}

export default SheetBuilder
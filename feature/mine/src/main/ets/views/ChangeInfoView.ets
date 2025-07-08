import { promptAction } from "@kit.ArkUI";
import { MKNavPathStack, PreferenceManagerFactory, putUserInfo } from "basic";

interface interUserInfo1 {
  "gender": string
}

interface interUserInfo2 {
  "username": string
}

interface interUserInfo3 {
  "phone": string
}

interface interUserInfo4 {
  "birth_date": Date
}

interface interUserInfo5 {
  "age": number
}

@Entry
@Component
struct ChangeInfoView {
  @State infoName: string = '';
  @State @Watch('onInput') username: string = ""
  @State @Watch('onInput') password: string = ""
  @State @Watch('onInput') phone: string = ''
  @State birth_date: Date = new Date() // 默认选中当前日期
  @State gender: string = '保密'
  @State @Watch('onInput') age: number = 0
  @State buttonEnable: boolean = false //按钮开关

  // 监听函数
  onInput() {
    this.buttonEnable = this.phone || this.password || this.username
      || this.age !== 0 || this.gender ? true : false
  }

  build() {
    NavDestination() {
      Column() {
        if (this.infoName === '用户名') {
          TextInput({ placeholder: '请输入用户名', text: $$this.username })
            .width('100%')
            .backgroundColor($r('app.color.white'))
            .margin({ top: 16 })
            .borderRadius(0)
        } else if (this.infoName === '性别') {
          Select([
            { value: '男' },
            { value: '女' },
            { value: '保密' }
          ])
            .menuAlign(MenuAlignType.CENTER)
            .optionWidth(200)
            .margin({ top: 16 })
            .height(30)
            .value('请选择性别')
            .width('100%')
            .backgroundColor($r('app.color.white'))
            .borderRadius(0)
            .onSelect((index: number) => {
              this.gender = ['男', '女', '保密'][index];
              console.log('选择的性别:', this.gender);
            })
        } else if (this.infoName === '年龄') {
          TextInput({ placeholder: '请输入年龄', text: $$this.age })
            .width('100%')
            .backgroundColor($r('app.color.white'))
            .margin({ top: 16 })
            .borderRadius(0)
            .type(InputType.Number)
        } else if (this.infoName === '手机号') {
          TextInput({ placeholder: '请输入手机号', text: $$this.phone })
            .width('100%')
            .backgroundColor($r('app.color.white'))
            .margin({ top: 16 })
            .borderRadius(0)
            .type(InputType.PhoneNumber)
        } else if (this.infoName === '生日') {
          DatePicker({
            start: new Date(1900, 1, 1), // 可选范围：起始年份
            end: new Date(), // 结束于今天
            selected: $$this.birth_date
          })
            .height(80)
            .layoutWeight(1)
            .selectedTextStyle({ font: { size: 14, weight: FontWeight.Medium } })
            .textStyle({ font: { size: 12, weight: FontWeight.Medium } })
            .lunar(false) // 关闭农历显示
        }
      }.height('100%').width('100%').backgroundColor($r('app.color.common_gray_bg'))
      .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.BOTTOM]) //隐藏状态和控制栏
    }.title(`设置${this.infoName}`).hideBackButton(true) //隐藏返回按钮
    .menus([{
      value: "确定", icon: 'resources/base/media/ic_public_save.svg',
      action: async () => {
        this.getUIContext().getFocusController().clearFocus(); //收起软键盘
        if (this.infoName === '用户名') {
          if (this.buttonEnable) {
            const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo');
            try {
              const userInfo: interUserInfo2 = {
                username: this.username
              }
              const res = await putUserInfo(userPref.getValue('userid'), userInfo)
              if (res.data.code === 200) {
                await userPref.setValue('username', this.username)
                AppStorage.setOrCreate('username', this.username);
                promptAction.openToast({ message: '设置成功' })
                MKNavPathStack.pop()
              }
            } catch (error) {
              console.error('设置错误：', error);
              if (error.response) {
                switch (error.response.status) {
                  case 400:
                    promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                    break;
                  case 401:
                    promptAction.openToast({ message: 'token无效' })
                    break;
                  case 403:
                    promptAction.openToast({ message: '无权限修改' })
                    break;
                  case 500:
                    promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                    break;
                  default:
                    promptAction.openToast({ message: '设置失败，请稍后重试' })
                    break;
                }
              }
            }
          } else {
            promptAction.openToast({ message: '请填写信息' })
          }
        } else if (this.infoName === '性别') {
          const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo');
          try {
            const userInfo: interUserInfo1 = {
              gender: this.gender
            }
            const res = await putUserInfo(userPref.getValue('userid'), userInfo)
            if (res.data.code === 200) {
              AppStorage.setOrCreate('gender', this.gender);
              promptAction.openToast({ message: '设置成功' })
              MKNavPathStack.pop()
            }
          } catch (error) {
            console.error('设置错误：', error);
            if (error.response) {
              switch (error.response.status) {
                case 400:
                  promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                  break;
                case 401:
                  promptAction.openToast({ message: 'token无效' })
                  break;
                case 403:
                  promptAction.openToast({ message: '无权限修改' })
                  break;
                case 500:
                  promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                  break;
                default:
                  promptAction.openToast({ message: '设置失败，请稍后重试' })
                  break;
              }
            }
          }
        } else if (this.infoName === '年龄') {
          if (this.buttonEnable) {
            const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo');
            try {
              const userInfo: interUserInfo5 = {
                age: this.age
              }
              const res = await putUserInfo(userPref.getValue('userid'), userInfo)
              if (res.data.code === 200) {
                AppStorage.setOrCreate('age', this.age);
                promptAction.openToast({ message: '设置成功' })
                MKNavPathStack.pop()
              }
            } catch (error) {
              console.error('设置错误：', error);
              if (error.response) {
                switch (error.response.status) {
                  case 400:
                    promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                    break;
                  case 401:
                    promptAction.openToast({ message: 'token无效' })
                    break;
                  case 403:
                    promptAction.openToast({ message: '无权限修改' })
                    break;
                  case 500:
                    promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                    break;
                  default:
                    promptAction.openToast({ message: '设置失败，请稍后重试' })
                    break;
                }
              }
            }
          } else {
            promptAction.openToast({ message: '请填写信息' })
          }
        } else if (this.infoName === '手机号') {
          if (this.buttonEnable) {
            const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo');
            try {
              const userInfo: interUserInfo3 = {
                phone: this.phone
              }
              const res = await putUserInfo(userPref.getValue('userid'), userInfo)
              if (res.data.code === 200) {
                AppStorage.setOrCreate('phone', this.phone);
                await userPref.setValue('phone', this.phone)
                promptAction.openToast({ message: '设置成功' })
                MKNavPathStack.pop()
              }
            } catch (error) {
              console.error('设置错误：', error);
              if (error.response) {
                switch (error.response.status) {
                  case 400:
                    promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                    break;
                  case 401:
                    promptAction.openToast({ message: 'token无效' })
                    break;
                  case 403:
                    promptAction.openToast({ message: '无权限修改' })
                    break;
                  case 500:
                    promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                    break;
                  default:
                    promptAction.openToast({ message: '设置失败，请稍后重试' })
                    break;
                }
              }
            }
          } else {
            promptAction.openToast({ message: '请填写信息' })
          }
        } else if (this.infoName === '生日') {
          const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo');
          try {
            const userInfo: interUserInfo4 = {
              birth_date: this.birth_date
            }
            const res = await putUserInfo(userPref.getValue('userid'), userInfo)
            if (res.data.code === 200) {
              AppStorage.setOrCreate('birth_date', this.birth_date);
              promptAction.openToast({ message: '设置成功' })
              MKNavPathStack.pop()
            }
          } catch (error) {
            console.error('设置错误：', error);
            if (error.response) {
              switch (error.response.status) {
                case 400:
                  promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                  break;
                case 401:
                  promptAction.openToast({ message: 'token无效' })
                  break;
                case 403:
                  promptAction.openToast({ message: '无权限修改' })
                  break;
                case 500:
                  promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                  break;
                default:
                  promptAction.openToast({ message: '设置失败，请稍后重试' })
                  break;
              }
            }
          }
        }
      }
    }])
    .onReady(async (context: NavDestinationContext) => {
      const pathStack = context.pathStack;
      const paramsArray = pathStack.getParamByName('ChangeInfoView') as string[]
      this.infoName = paramsArray[0];
    })
  }
}

export default ChangeInfoView
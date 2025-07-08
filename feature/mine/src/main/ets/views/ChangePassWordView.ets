import { getRegisterApi, MKNavPathStack, PreferenceManagerFactory, putUserInfo } from "basic";
import { promptAction } from "@kit.ArkUI";
import MqttClientWrapper from "basic/src/main/ets/utils/MyMqttClient";
import { updateUserInfo } from "../pages";

interface interUserInfo {
  "password": string
}

@Entry
@Component
struct ChangePassWordView {
  @State @Watch('onInput') phone: string = '';
  @State @Watch('onInput') password: string = ''
  @State valid: boolean = false

  onInput() {
    this.valid = this.phone && this.password ? true : false
  }

  build() {
    Column({ space: 18 }) {
      TextInput({ placeholder: '请输入正确格式手机号', text: $$this.phone }).width('90%')
        .type(InputType.PhoneNumber)
      TextInput({ placeholder: '请输入修改密码', text: $$this.password }).width('90%')
        .type(InputType.Password)
      Button('修改密码', { stateEffect: true }).width('80%').enabled(this.valid)
        .onClick(async () => {
          this.getUIContext().getFocusController().clearFocus(); //收起软键盘
          const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo');
          if (userPref.getValue('userid') === '' || userPref.getValue('userid') === null) {
            try {
              const res = await getRegisterApi(this.phone)
              if (res.data.code === 200) {
                const changePassWord: interUserInfo = {
                  password: this.password
                }
                try {
                  const changePassWordRes = await putUserInfo(res.data.data.id, changePassWord)
                  if (changePassWordRes.data.code === 200) {
                    promptAction.openToast({ message: '修改密码成功' })
                    MKNavPathStack.pop()
                    MKNavPathStack.popToName('ChangePassWordView')
                  }
                } catch (error) {
                  console.error('修改错误：', error);
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
                        promptAction.openToast({ message: '修改失败，请稍后重试' })
                        break;
                    }
                  }
                }
              }
            } catch (error) {
              console.error('修改密码查询id错误：', error);
              if (error.response) {
                switch (error.response.status) {
                  case 400:
                    promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                    break;
                  case 4004:
                    promptAction.openToast({ message: '用户未找到，请检查输入信息' })
                    break;
                  case 500:
                    promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                    break;
                  default:
                    promptAction.openToast({ message: '修改失败，请稍后重试' })
                    break;
                }
              }
            }
          } else {
            if (this.phone === userPref.getValue('phone')) {
              try {
                const changePassWord: interUserInfo = {
                  password: this.password
                }
                const changePassWordRes = await putUserInfo(userPref.getValue('userid'), changePassWord)
                if (changePassWordRes.data.code === 200) {
                  await userPref.deleteValue('token');
                  await userPref.deleteValue('userid');
                  await userPref.deleteValue('phone');
                  await userPref.deleteValue('username');
                  await userPref.deleteValue('avatar_url');
                  await userPref.deleteValue('token_expire');
                  promptAction.openToast({ message: '已退出登录' });
                  AppStorage.setOrCreate('isLoggedIn', false); //将登陆状态置为关
                  MqttClientWrapper.destroy() //销毁客户端
                  updateUserInfo(); // 退出登录后立即更新 UI
                  MKNavPathStack.replacePathByName('LoginView', null)
                  MKNavPathStack.removeByName('InstallView')
                  promptAction.openToast({ message: '修改密码成功' })
                }
              } catch (error) {
                console.error('修改错误：', error);
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
                      promptAction.openToast({ message: '修改失败，请稍后重试' })
                      break;
                  }
                }
              }
            } else {
              promptAction.openToast({ message: '手机号不存在' })
            }
          }
        })
    }.width('100%').height('100%').padding({ top: 20 })
    .justifyContent(FlexAlign.Center)
  }
}

export default ChangePassWordView
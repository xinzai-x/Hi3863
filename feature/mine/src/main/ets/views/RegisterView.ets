import { Loading, MKNavPathStack, postRegisterApi } from "basic"
import { promptAction } from "@kit.ArkUI"

interface interUserInfo {
  "username": string,
  "password": string,
  "phone": string,
  "gender": string,
  "birth_date": Date,
  "age": number
}

@Entry
@Component
struct RegisterView {
  @State @Watch('onInput') username: string = ""
  @State @Watch('onInput') password: string = ""
  @State @Watch('onInput') phone: string = ''
  @State selectedDate: Date = new Date() // 默认选中当前日期
  @State gender: string = ''
  @State age: number = 0 // 存储计算后的年龄
  @State buttonEnable: boolean = false //按钮开关
  //加载动画组件
  dialog = new CustomDialogController({
    builder: Loading({ message: '注册中 ...' }),
    customStyle: true, // 自定义样式
    alignment: DialogAlignment.Center, // 居中
    autoCancel: false //禁止点击其它区域关闭加载弹出
  })

  // 监听函数
  onInput() {
    this.buttonEnable = this.phone && this.password && this.username
      && this.gender ? true : false
  }

  // 计算年龄的函数
  private calculateAge(birthDate: Date): number {
    const today = new Date()
    let age = today.getFullYear() - birthDate.getFullYear()

    // 处理月份和日期的边界情况（如果今年生日还没到，年龄减1）
    const monthDiff = today.getMonth() - birthDate.getMonth()
    if (monthDiff < 0 || (monthDiff === 0 && today.getDate() < birthDate.getDate())) {
      age--
    }
    return age
  }

  build() {
    Scroll() {
      Column({ space: 16 }) {
        Text('注册账号').fontSize(24).fontWeight(500)

        TextInput({ placeholder: '请输入用户名', text: $$this.username }).placeholderFont({ size: 14 })
          .width('100%')

        Row() {
          Text('性别：').fontWeight(FontWeight.Medium)
          Select([
            { value: '男' },
            { value: '女' },
            { value: '保密' }
          ]).layoutWeight(1)
            .value('请选择性别')
            .onSelect((index: number) => {
              this.gender = ['男', '女', '保密'][index];
              console.log('选择的性别:', this.gender);
            })
        }.width('100%')

        TextInput({ placeholder: '请输入手机号', text: $$this.phone })
          .type(InputType.PhoneNumber).placeholderFont({ size: 14 }).width('100%')

        TextInput({ placeholder: '请输入密码,6位以上', text: $$this.password }).width('100%')
          .type(InputType.Password).placeholderFont({ size: 14 })

        Row() {
          Text('生日：').textAlign(TextAlign.Start)
          DatePicker({
            start: new Date(1900, 1, 1), // 可选范围：起始年份
            end: new Date(), // 结束于今天
            selected: $$this.selectedDate
          })
            .height(80)
            .layoutWeight(1)
            .selectedTextStyle({ font: { size: 14, weight: FontWeight.Medium } })
            .textStyle({ font: { size: 12, weight: FontWeight.Medium } })
            .lunar(false) // 关闭农历显示
        }.width('100%')

        Button('注册', { stateEffect: true }).width('90%').enabled(this.buttonEnable)
          .onClick(async () => {
            this.dialog.open()
            setTimeout(async () => {
              try {
                const userInfo: interUserInfo = {
                  username: this.username,
                  password: this.password,
                  phone: this.phone,
                  birth_date: this.selectedDate,
                  gender: this.gender,
                  age: this.calculateAge(this.selectedDate)
                }
                console.log('用户信息：', JSON.stringify(userInfo))
                const res = await postRegisterApi(userInfo)

                if (res.data.code === 201) {
                  promptAction.openToast({ message: "注册成功" })
                  console.info('注册信息：', JSON.stringify(res))
                  this.dialog.close() //关闭动画
                  MKNavPathStack.pop(); //返回上一页
                } else {
                  // 处理业务错误（如果服务器返回200但业务失败）
                  this.dialog.close() //关闭动画
                  promptAction.openToast({ message: res.data.message || '注册失败' })
                }
              } catch (error) {
                // 处理HTTP错误状态码
                console.error('注册错误：', error);
                this.dialog.close() //关闭动画

                if (error.response) {
                  switch (error.response.status) {
                    case 400:
                      promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                      break;
                    case 500:
                      promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                      break;
                    default:
                      promptAction.openToast({ message: '注册失败，请稍后重试' })
                      break;
                  }
                }
              }
            }, 500)
          })
      }
      .width('100%')
      .height(460)
      .backgroundColor($r('app.color.white'))
      .borderRadius(16)
      .padding(16)
    }
    .padding(16)
    .width('100%')
    .height('100%')
    .backgroundImage('https://i.postimg.cc/Pfz633xS/register-Image.png')
    .backgroundImageSize(ImageSize.FILL)
    .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.BOTTOM])
  }
}

export default RegisterView
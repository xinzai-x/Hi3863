import RegisterView from '../views/RegisterView'
import LoginView from '../views/LoginView'
import UserInfoView from '../views/UserInfoView'
import ChangeInfoView from '../views/ChangeInfoView'
import ChangePassWordView from '../views/ChangePassWordView'
import InstallView from '../views/InstallView'
import AIView from '../views/AIView'

@Builder
function LoginViewBuilder() {
  NavDestination() {
    LoginView()
  }
  .hideTitleBar(true)
}

@Builder
function RegisterViewBuilder() {
  NavDestination() {
    RegisterView()
  }.title('注册账号')
  .hideBackButton(true)
}


@Builder
function UserInfoViewBuilder() {
  NavDestination() {
    UserInfoView()
  }.title('个人信息')
}

@Builder
function ChangeInfoViewBuilder() {
  ChangeInfoView()
}

@Builder
function ChangePassWordViewBuilder() {
  NavDestination() {
    ChangePassWordView()
  }.title('修改密码').hideBackButton(true)
}

@Builder
function InstallViewBuilder() {
  NavDestination() {
    InstallView()
  }.title('设置')
}

@Builder
function AIViewBuilder() {
  NavDestination() {
    AIView()
  }.title('智能客服')
  .width('100%').height('100%')
}
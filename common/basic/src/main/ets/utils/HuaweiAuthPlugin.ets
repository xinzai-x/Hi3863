import { authentication } from '@kit.AccountKit'

class HuaweiAuthPlugin {
  async requestAuth() {
    // 1. 创建一个Account Kit授权请求对象，可通过返回值设置请求参数。
    const huaweiIdProvider = new authentication.HuaweiIDProvider()
    const authCreateRequest = huaweiIdProvider.createAuthorizationWithHuaweiIDRequest()
    // 2. 添加请求参数
    authCreateRequest.scopes = ['phone']
    authCreateRequest.permissions = ['serviceauthcode']
    authCreateRequest.forceAuthorization = true
    // 3. 执行授权请求，获取认证码
    const authController = new authentication.AuthenticationController(getContext())
    const authResponse: authentication.AuthorizationWithHuaweiIDResponse =
      await authController.executeRequest(authCreateRequest)
    const authorizationCode = authResponse.data?.authorizationCode
    return authorizationCode
  }

  // getHuaweiIDState  api12 支持
  async cancelAuth() {
    try {
      // 1. 创建一个Account Kit授权请求对象，可通过返回值设置请求参数。
      const huaweiIdProvider = new authentication.HuaweiIDProvider()
      const authCancelRequest = huaweiIdProvider.createCancelAuthorizationRequest()
      // 2. 取消授权
      const authController = new authentication.AuthenticationController(getContext(this))
      await authController.executeRequest(authCancelRequest)
      return true
    } catch (e) {
      console.log('mk-logger', JSON.stringify(e))
      return false
    }
  }
}

export const huaweiAuthPlugin = new HuaweiAuthPlugin()
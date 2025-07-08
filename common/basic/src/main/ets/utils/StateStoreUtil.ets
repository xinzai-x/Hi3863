//全局的UI状态存储
// 弹窗UI状态级及数据
function PopInit() {
  const popInfoInit: Record<string, boolean> = {
    'isShowSheet': true, //半模态开关
  }
  //创建一个全局保存的对象popInfo，将创建好的初始化对象值赋值给popInfo
  AppStorage.setOrCreate('popInfo', popInfoInit)
}

export default PopInit
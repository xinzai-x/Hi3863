import http from '@ohos.net.http';

interface ChatMessage {
  id: number;
  text: string;
  isUser: boolean;
  timestamp: string;
}

// 在api-types.d.ts中定义
interface QingyunkeResponse {
  result: number; // 0表示成功，非0表示失败
  content: string; // 返回的聊天内容
}

@Entry
@Component
struct AIView {
  @State message: string = '';
  @State chatList: ChatMessage[] = [];
  @State isLoading: boolean = false;
  private scroller: Scroller = new Scroller();

  scrollToBottom() {
    this.scroller.scrollEdge(Edge.Bottom);
  }

  aboutToAppear() {
    // Add welcome message
    const welcomeMessage: ChatMessage = {
      id: 1,
      text: '你好！我是AI助手，有什么可以帮助你的吗？',
      isUser: false,
      timestamp: getCurrentTime()
    };
    this.chatList.push(welcomeMessage);
  }

  build() {
    Column() {
      // Chat messages area
      Scroll(this.scroller) {
        Column({ space: 15 }) {
          ForEach(this.chatList, (item: ChatMessage) => {
            Row({ space: 10 }) {
              if (!item.isUser) {
                Image('https://i.postimg.cc/V644G2Nn/icon-foreground.png').width(40).height(40)
              }

              Column() {
                Text(item.text)
                  .padding(12)
                  .backgroundColor(item.isUser ? '#007AFF' : $r('app.color.white'))
                  .fontColor(item.isUser ? Color.White : Color.Black)
                  .borderRadius(18)
                  .maxLines(100)
                  .textOverflow({ overflow: TextOverflow.None })
                  .fontSize(16)
                  .lineHeight(22)

                Text(item.timestamp)
                  .fontSize(12)
                  .fontColor('#999999')
                  .margin({ top: 5 })
                  .alignSelf(item.isUser ? ItemAlign.End : ItemAlign.Start)
              }
              .alignItems(item.isUser ? HorizontalAlign.End : HorizontalAlign.Start)
              .layoutWeight(1)

              if (item.isUser) {
                // User avatar
                Image('https://i.postimg.cc/SNpDZvSY/customer.png').width(40).height(40)
              }
            }
            .width('100%')
            .alignItems(VerticalAlign.Top)
            .justifyContent(item.isUser ? FlexAlign.End : FlexAlign.Start)
          }, (item: ChatMessage) => item.id.toString())

          // Loading indicator
          if (this.isLoading) {
            Row({ space: 10 }) {
              Image('https://i.postimg.cc/V644G2Nn/icon-foreground.png').width(40).height(40) //AI

              Row() {
                ForEach([1, 2, 3], (item: number) => {
                  Circle({ width: 8, height: 8 })
                    .fill('#999999')
                    .margin({ right: 4 })
                    .animation({
                      duration: 600,
                      iterations: -1,
                      playMode: PlayMode.Alternate
                    })
                    .scale({ x: 0.5, y: 0.5 })
                })
              }
              .padding(12)
              .backgroundColor('#E5E5EA')
              .borderRadius(18)
            }
            .width('100%')
            .justifyContent(FlexAlign.Start)
          }
        }
        .padding(15).align(Alignment.TopStart)
        .width('100%')
      }
      .layoutWeight(1)
      .scrollBar(BarState.Off)
      .edgeEffect(EdgeEffect.Spring)
      .backgroundColor('#F2F2F7')

      // Bottom input area
      Row() {
        TextInput({
          placeholder: '输入消息...',
          text: this.message
        })
          .layoutWeight(1)
          .height(44)
          .padding({ left: 15, right: 15 })
          .backgroundColor(Color.White)
          .borderRadius(22)
          .border({ width: 1, color: '#E5E5EA' })
          .fontSize(16)
          .onChange((value: string) => {
            this.message = value;
          })
          .onSubmit(() => {
            this.sendMessage();
          })

        Button('发送')
          .width(80)
          .height(44)
          .margin({ left: 10 })
          .borderRadius(22)
          .backgroundColor(this.message.trim() ? '#007AFF' : '#C7C7CC')
          .fontColor(Color.White)
          .fontSize(16)
          .enabled(this.message.trim() !== '' && !this.isLoading)
          .onClick(() => {
            this.sendMessage();
          })
      }
      .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.BOTTOM]) //隐藏状态和控制栏
      .width('100%')
      .padding({
        left: 15,
        right: 15,
        top: 10,
        bottom: 10
      })
      .backgroundColor(Color.White)
      .border({ width: { top: 1 }, color: '#E5E5EA' })
    }
    .height('100%')
    .width('100%')
    .backgroundColor('#F2F2F7')
    .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.TOP, SafeAreaEdge.BOTTOM])
  }

  sendMessage() {
    if (this.message.trim() === '') {
      return;
    }

    const userMessage: ChatMessage = {
      id: this.chatList.length + 1,
      text: this.message,
      isUser: true,
      timestamp: getCurrentTime()
    };
    this.chatList.push(userMessage);
    this.message = '';
    this.isLoading = true;

    // Scroll to bottom
    // 正确用法 - 滚动到底部
    this.scroller.scrollEdge(Edge.Bottom);

    // Simulate AI response
    this.getAIResponse(userMessage.text);
  }

  async getAIResponse(question: string) {
    try {
      const httpRequest = http.createHttp();
      const response = await httpRequest.request(
        `http://api.qingyunke.com/api.php?key=free&appid=0&msg=${encodeURIComponent(question)}`
      );
      const data: QingyunkeResponse = JSON.parse(response.result as string);
      const aiMessage: ChatMessage = {
        id: this.chatList.length + 1,
        text: data.content,
        isUser: false,
        timestamp: getCurrentTime()
      };
      this.chatList.push(aiMessage);

      // Scroll to bottom
      this.scroller.scrollEdge(Edge.Bottom);
    } catch (error) {
      console.error('Error fetching AI response:', error);
      const errorMessage: ChatMessage = {
        id: this.chatList.length + 1,
        text: 'AI助手暂时无法响应，请稍后再试。',
        isUser: false,
        timestamp: getCurrentTime()
      };
      this.chatList.push(errorMessage);
    } finally {
      this.isLoading = false;
    }
  }
}

function getCurrentTime(): string {
  const now = new Date();
  const hours = now.getHours().toString().padStart(2, '0');
  const minutes = now.getMinutes().toString().padStart(2, '0');
  return `${hours}:${minutes}`;
}

export default AIView
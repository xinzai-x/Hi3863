import { preferences } from '@kit.ArkData';

/**
 * 偏好设置管理类，封装对本地持久化存储（preferences）的增删改查操作
 * 使用示例：
 * const prefs = new PreferenceManager(store, "user_settings");
 * await prefs.setValue("token", "abc123");
 */
export class PreferenceManager {
  private store: preferences.Preferences; // 实际的存储实例
  private name: string; // 存储仓库名称（用于删除整个仓库时识别）

  /**
   * 构造函数
   * @param store 已初始化的 preferences 存储实例
   * @param name 当前存储仓库名称（需唯一）
   */
  constructor(store: preferences.Preferences, name: string) {
    this.store = store;
    this.name = name;
  }

  /**
   * 存储键值对（同步写入内存 + 异步刷盘）
   * @param key 键名
   * @param value 值（字符串类型）
   * @returns Promise<void> 刷盘完成后 resolve
   */
  async setValue(key: string, value: string): Promise<void> {
    this.store.putSync(key, value); // 同步写入内存
    await this.store.flush(); // 异步持久化到磁盘
  }

  /**
   * 读取键对应的值
   * @param key 键名
   * @param defaultValue 可选默认值（未找到时返回）
   * @returns 存储的值或默认值（字符串）
   */
  getValue(key: string, defaultValue: string = ""): string {
    return this.store.getSync(key, defaultValue) as string;
  }

  /**
   * 删除指定键值对（同步删除 + 异步刷盘）
   * @param key 键名
   * @returns Promise<void> 刷盘完成后 resolve
   */
  async deleteValue(key: string): Promise<void> {
    this.store.deleteSync(key); // 同步删除
    await this.store.flush(); // 异步持久化
  }

  /**
   * 删除整个存储仓库（危险操作！）
   * @returns Promise<void> 删除完成后 resolve
   */
  async deleteStorage(): Promise<void> {
    await preferences.deletePreferences(null, { name: this.name });
  }
}

/**
 * PreferenceManager 的工厂类（单例模式），用于集中管理多个存储仓库
 * 使用示例：
 * PreferenceManagerFactory.getInstance().init(context);
 * const prefs = factory.getPreference("app_config");
 */
export class PreferenceManagerFactory {
  private static instance: PreferenceManagerFactory; // 单例实例
  private context: Context | null = null; // 应用上下文（必须初始化）
  private stores: Map<string, PreferenceManager> = new Map(); // 存储多个仓库实例

  private constructor() {} // 禁止外部 new 实例

  /**
   * 获取工厂单例
   * @returns 全局唯一的 PreferenceManagerFactory 实例
   */
  public static getInstance(): PreferenceManagerFactory {
    if (!PreferenceManagerFactory.instance) {
      PreferenceManagerFactory.instance = new PreferenceManagerFactory();
    }
    return PreferenceManagerFactory.instance;
  }

  /**
   * 初始化工厂（必须在获取实例前调用）
   * @param context 应用上下文
   */
  public init(context: Context): void {
    this.context = context;
  }

  /**
   * 获取指定名称的存储管理器（如果不存在则创建）
   * @param name 存储仓库名称
   * @throws 未初始化 context 时抛出错误
   * @returns PreferenceManager 实例
   */
  public getPreference(name: string): PreferenceManager {
    if (!this.context) {
      throw new Error("PreferenceManagerFactory not initialized. Call init(context) first.");
    }

    // 缓存检查：避免重复创建同名仓库
    if (!this.stores.has(name)) {
      const store = preferences.getPreferencesSync(this.context, { name });
      this.stores.set(name, new PreferenceManager(store, name));
    }

    return this.stores.get(name)!; // 非空断言（因为必定存在）
  }
}
import { BusinessError } from '@kit.BasicServicesKit';
import { relationalStore } from '@kit.ArkData';

type DBRow = Record<string, string | number | null>;

// ArkTS 中避免联合类型，使用明确类型
type QueryOperator =
  | 'equalTo' // 等于，常用于精确匹配，例如 NAME = '张三'
  | 'like' // 模糊匹配，常用于字符串搜索，例如 NAME LIKE '%张%'
  | 'greaterThan' // 大于，例如 AGE > 18
  | 'lessThan' // 小于，例如 AGE < 60
  | 'greaterThanOrEqualTo' // 大于等于，例如 AGE >= 30
  | 'lessThanOrEqualTo'; // 小于等于，例如 AGE <= 50

/**
 * 查询条件结构定义
 */
interface QueryCondition {
  field: string; // 查询字段，如 'NAME'、'AGE'
  operator?: QueryOperator; // 查询操作符，默认为 'equalTo'
  value: string | number; // 查询值（ArkTS 中不建议写联合类型，但此处使用时需确保类型一致）
}

class DBUtil {
  private rdbStore: relationalStore.RdbStore | null = null;

  /**
   * 初始化数据库，仅建立连接，不建表，调用时可能需要配置一个变量防止每次都创建
   * @param context 应用上下文
   * @param storeConfig 数据库配置项
   * @param onSuccess 初始化成功后的回调,方便顺便建表
   */
  initDB(context: Context, storeConfig: relationalStore.StoreConfig, onSuccess?: () => void): void {
    relationalStore.getRdbStore(context, storeConfig, (err, store) => {
      if (err) {
        console.error(`[RdbStore_getRdbStore] failed. Code: ${err.code}, message: ${err.message}`);
        return;
      }
      console.log(`[RdbStore_getRdbStore] success.`);
      this.rdbStore = store;
      if (onSuccess) {
        onSuccess();
      }
    });
  }

  /**
   * 创建表（需先调用 initDB 后再使用）
   * @param createTableSQL 建表 SQL 语句
   */
  createTable(createTableSQL: string): void {
    if (!this.rdbStore) {
      console.error('RdbStore is not initialized. Please call initDB first.');
      return;
    }

    this.rdbStore.executeSql(createTableSQL, [], (err) => {
      if (err) {
        console.error(`[RdbStore_executeSql] create table failed. Code: ${err.code}, message: ${err.message}`);
        return;
      }
      console.log(`[RdbStore_executeSql] create table success.`);
    });
  }

  /**
   * 插入数据
   * @param tableName 表名
   * @param value 插入的键值对（ValuesBucket）
   * @returns 插入的行 ID，失败返回 -1
   */
  async insertData(tableName: string, value: relationalStore.ValuesBucket): Promise<number> {
    if (this.rdbStore !== null) {
      try {
        console.info(`[RdbStore_insert] 尝试插入表 ${tableName} 数据: ${JSON.stringify(value)}`);
        const rowId = await this.rdbStore.insert(tableName, value);

        if (rowId === -1) {
          console.error(`[RdbStore_insert] 插入失败（未抛异常），rowId = -1，可能存在字段类型错误或约束冲突`);
        }

        return rowId;
      } catch (e) {
        const err = e as BusinessError;
        console.error(`[RdbStore_insert] ${tableName} failed. Code: ${err.code}, message: ${err.message}`);
      }
    } else {
      console.error(`[RdbStore_insert] rdbStore 未初始化`);
    }
    return -1;
  }

  /**
   * 通用查询方法（支持多字段条件）
   * @param column 查询字段列，如 ['ID', 'NAME']
   * @param tableName 表名
   * @param conditions 可选条件数组，如 [{ field: 'NAME', operator: 'equalTo', value: '张三' }]
   * @returns 返回原始数据对象数组
   */
  async queryDB(
    column: Array<string>,
    tableName: string,
    conditions?: Array<QueryCondition>
  ): Promise<Array<DBRow>> {
    const predicates = new relationalStore.RdbPredicates(tableName);

    // 构造查询条件
    if (conditions) {
      for (const condition of conditions) {
        const field: string = condition.field;
        const operator: QueryOperator = condition.operator ?? 'equalTo';
        const value: string | number = condition.value;

        // 用 switch 方式判断操作符，ArkTS 不支持动态函数调用
        switch (operator) {
          case 'equalTo':
            predicates.equalTo(field, value);
            break;
          case 'like':
            if (typeof value === 'string') {
              predicates.like(field, value);
            } else {
              console.warn(`[queryDB] 'like' operator requires string value, got: ${typeof value}`);
            }
            break;
          case 'greaterThan':
            predicates.greaterThan(field, value);
            break;
          case 'lessThan':
            predicates.lessThan(field, value);
            break;
          case 'greaterThanOrEqualTo':
            predicates.greaterThanOrEqualTo(field, value);
            break;
          case 'lessThanOrEqualTo':
            predicates.lessThanOrEqualTo(field, value);
            break;
          default:
            console.warn(`[queryDB] Unsupported operator: ${operator}`);
        }
      }
    }

    const resultList: Array<DBRow> = [];

    // 查询执行
    if (this.rdbStore !== null) {
      try {
        const resultSet = await this.rdbStore.query(predicates, column);
        console.info(`[queryDB] start query from ${tableName}`);

        while (resultSet.goToNextRow()) {
          const row: DBRow = {};
          for (const col of column) {
            const index = resultSet.getColumnIndex(col);
            try {
              row[col] = resultSet.getString(index);
            } catch {
              try {
                row[col] = resultSet.getDouble(index);
              } catch {
                try {
                  row[col] = resultSet.getLong(index);
                } catch {
                  row[col] = null;
                }
              }
            }
          }
          resultList.push(row);
        }

        resultSet.close();
      } catch (e) {
        const err = e as BusinessError;
        console.error(`[RdbStore_query] ${tableName} failed. Code: ${err.code}, message: ${err.message}`);
      }
    }

    return resultList;
  }

  /**
   * 根据 ID 删除指定记录
   * @param tableName 表名
   * @param id 主键 ID
   */
  deleteDataByID(tableName: string, id: number | string): void {
    const predicates = new relationalStore.RdbPredicates(tableName).equalTo('ID', id);
    if (this.rdbStore !== null) {
      this.rdbStore.delete(predicates, (err, rows) => {
        if (err) {
          console.error(`Delete failed. Code: ${err.code}, message: ${err.message}`);
          return;
        }
        console.info(`Deleted rows: ${rows}`);
      });
    }
  }

  /**
   * 根据 ID 更新记录
   * @param tableName 表名
   * @param id 主键 ID
   * @param value 要更新的字段值（ValuesBucket）
   */
  async updateDataByID(tableName: string, id: string, value: relationalStore.ValuesBucket): Promise<void> {
    const predicates = new relationalStore.RdbPredicates(tableName).equalTo('ID', id);

    if (this.rdbStore !== null) {
      try {
        const rows = await this.rdbStore.update(value, predicates);
        console.info(`[更新成功] 表: ${tableName}, ID: ${id}, 更新字段: ${JSON.stringify(value)}, 影响行数: ${rows}`);
      } catch (err) {
        console.error(`[更新失败] 表: ${tableName}, ID: ${id}, 错误码: ${err.code}, 错误信息: ${err.message}`);
      }
    } else {
      console.warn(`[警告] rdbStore 尚未初始化，无法更新 表: ${tableName}`);
    }
  }

  // 自动判断是否需要 VACUUM 的实现
  async autoVacuumIfNeeded(): Promise<void> {
    if (!this.rdbStore) {
      console.warn('[DBUtil_autoVacuumIfNeeded] rdbStore not initialized.');
      return;
    }

    try {
      let pageCount = 0;
      let freeCount = 0;

      // 查询 page_count
      const pageResultSet = await this.rdbStore.querySql('PRAGMA page_count;', []);
      if (pageResultSet.goToFirstRow()) {
        const index = pageResultSet.getColumnIndex('page_count');
        pageCount = pageResultSet.getLong(index);
      }
      pageResultSet.close();

      // 查询 freelist_count
      const freeResultSet = await this.rdbStore.querySql('PRAGMA freelist_count;', []);
      if (freeResultSet.goToFirstRow()) {
        const index = freeResultSet.getColumnIndex('freelist_count');
        freeCount = freeResultSet.getLong(index);
      }
      freeResultSet.close();

      const freeRatio: number = pageCount > 0 ? freeCount / pageCount : 0;

      console.info(`[DBUtil_autoVacuumIfNeeded] pageCount=${pageCount}, freeCount=${freeCount}, freeRatio=${freeRatio}`);

      if (pageCount > 1000 && freeRatio > 0.3) {
        console.warn('[DBUtil_autoVacuumIfNeeded] Triggering VACUUM...');
        await new Promise<void>((resolve, reject) => {
          this.rdbStore!.executeSql('VACUUM;', [], (err) => {
            if (err) {
              reject(err);
            } else {
              resolve();
            }
          });
        });
        console.info('[DBUtil_autoVacuumIfNeeded] VACUUM completed.');
      } else {
        console.info('[DBUtil_autoVacuumIfNeeded] No need to VACUUM.');
      }
    } catch (e) {
      const err = e as BusinessError;
      console.error(`[DBUtil_autoVacuumIfNeeded] Failed. Code: ${err.code}, message: ${err.message}`);
    }
  }
}

const dbUtil = new DBUtil();

export { dbUtil };
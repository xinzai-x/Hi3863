import axios, { AxiosError, AxiosRequestConfig, AxiosResponse, InternalAxiosRequestConfig } from '@ohos/axios'
import { PreferenceManagerFactory } from '..'

//API发请求封装
// 实例化 通用配置
const axiosInstance = axios.create({
  //http://192.168.43.10:10031/api/v1
  baseURL: 'http://110.42.63.197:24188/api/v1',
  timeout: 20000
})

// 后端返回的数据格式 code, msg, result
export interface HttpResponse<T> {
  code: number
  message: string
  data: T
}

// AxiosResponse 是 axios 响应类型
export type ResponseType<T> = AxiosResponse<HttpResponse<T>>

export interface CustomAxiosRequestConfig extends AxiosRequestConfig {
  needToken?: boolean
}

// 添加请求拦截器
axiosInstance.interceptors.request.use((config: InternalAxiosRequestConfig) => {
  const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo')
  const token = userPref.getValue('token')
  if (token && token.trim() !== '') {
    config.headers['Authorization'] = `Bearer ${token}`;
  }
  return config;
}, (error: AxiosError) => {
  // 对请求错误做些什么
  console.error('请求拦截器错误：', error);
  return Promise.reject(error);
});

// 添加响应拦截器
axiosInstance.interceptors.response.use((response: AxiosResponse) => {
  // 对响应数据做点什么
  return response;
}, (error: AxiosError): Promise<AxiosError> => {
  // 对响应错误做点什么
  return Promise.reject(error);
});


export class RequestAxios {
  //获取数据
  static get<T>(url: string, params?: object): Promise<ResponseType<T>> {
    return axiosInstance.get<null, ResponseType<T>>(url, { params })
  }

  static post<T>(url: string, data?: object): Promise<ResponseType<T>> {
    return axiosInstance.post<null, ResponseType<T>>(url, data);
  }

  static delete<T>(url: string, data?: object): Promise<ResponseType<T>> {
    return axiosInstance.delete<null, ResponseType<T>>(url, data)
  }

  static put<T>(url: string, data?: object): Promise<ResponseType<T>> {
    return axiosInstance.put<null, ResponseType<T>>(url, data)
  }
}

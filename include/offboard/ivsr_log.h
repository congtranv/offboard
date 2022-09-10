#ifndef IVSR_LOG_H_
#define IVSR_LOG_H_

void ivsrLogInfo(const char* format, ...);
void ivsrLogWarn(const char* format, ...);
void ivsrLogDebug(const char* format, ...);
void ivsrLogError(const char* format, ...);

#define IVSR_LOGI(...) ivsrLogInfo(__VA_ARGS__)
#define IVSR_LOGW(...) ivsrLogWarn(__VA_ARGS__)
#define IVSR_LOGD(...) ivsrLogDebug(__VA_ARGS__)
#define IVSR_LOGE(...) ivsrLogError(__VA_ARGS__)

#endif  // IVSR_LOG_H_

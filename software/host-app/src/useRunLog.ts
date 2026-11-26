import { onMounted, ref } from 'vue';
import { desktop, logApi, type LogLevel } from './api';

export function useRunLog() {
  const logs = ref<{ time: string; message: string; level: LogLevel }[]>([]);
  const logPath = ref('');
  const storageError = ref('');

  function log(message: string, level: LogLevel = 'info') {
    logs.value.push({
      time: new Date().toLocaleTimeString('zh-CN', { hour12: false }),
      message,
      level,
    });
    if (logs.value.length > 200) logs.value.shift();
    // Keep the UI usable on a full disk. Never log a writer error through the writer itself.
    if (desktop) {
      void logApi.append(level, message).catch((e) => {
        storageError.value = `日志未能保存：${e}`;
      });
    }
  }

  onMounted(() => {
    if (desktop) {
      void logApi
        .path()
        .then((path) => {
          logPath.value = path;
        })
        .catch((e) => {
          storageError.value = `文件日志不可用：${e}`;
        });
    }
  });

  async function openLogFolder() {
    try {
      await logApi.openFolder();
    } catch (e) {
      log(`打开日志目录失败：${e}`, 'error');
    }
  }

  return { logs, log, logPath, storageError, openLogFolder };
}

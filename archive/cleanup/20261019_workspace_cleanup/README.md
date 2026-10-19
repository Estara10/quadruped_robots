# 2026-10-19 工作区清理记录

按用户授权删除logs中的临时overlay资源、整个logs/manual_abcd_compare（包含最近D组观看）、五处子项目.git_backup。当前主仓库.git未删除。

非手动测试目录的PPM/BMP截图已转换为PNG，并核对尺寸与RGBA像素一致后删除源图；转换失败项保留源文件。手动测试目录按授权整体删除，不保留其截图和逐帧记录。

overlay配置文本及模型路径/哈希清单见本目录；另按内容去重保留模型副本，以免丢失overlay内独有版本。删除的overlay不再可直接恢复运行，须按配置重新准备。历史报告中的旧overlay、手动测试、PPM/BMP路径可能失效，图片替代路径见image_conversion.json。

T1/T2等非手动测试批次的runtime_record、context、terminal和指标未删除。未运行仿真、测试、训练，未提交Git。

具体删除范围、计数和空间变化见cleanup_summary.json。

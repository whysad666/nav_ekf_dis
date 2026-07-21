# 协同测距与组合导航近期实验分析报告

## 文件

- `协同测距与组合导航近期实验分析报告_1000字.docx`：约1000字的Word/WPS可编辑精简版。
- `协同测距与组合导航近期实验分析报告_1000字.md`：内容相同的Markdown复制版本，保留公式块。
- `协同测距与组合导航近期实验分析报告.pdf`：17页A4正式报告。
- `协同测距与组合导航近期实验分析报告_可编辑.docx`：Word/WPS可编辑复制版本，内嵌8张图片。
- `nav_uwb_recent_experiments.tex`：可编辑LaTeX源文件。
- `nav_uwb_recent_experiments_editable.html`：Word版本的可编辑中间源文件。
- `figures/`：报告引用的实验图件副本。

## 编译

在本目录运行：

```bash
XDG_CACHE_HOME=/home/whysad/实验数据处理/tools/tectonic/cache \
  ../../tools/tectonic/tectonic --keep-logs nav_uwb_recent_experiments.tex
```

## 结果边界

报告将可确认的MEMS+GNSS、纯惯性和UWB精度实验用于正文，将range发散结果仅作为失败机理分析。L3高精度结果是同一导航程序使用高精度IMU重新解算的相对参考，不是独立测量真值。

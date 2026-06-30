# r2_pingmu

比赛屏幕显示页面，用于切换红蓝背景、选择场地方块颜色，并把确认后的 KFS 分布保存到本地 JSON 文件。



## 运行方法

进入项目目录：

```bash
cd /home/rc4/r2_pingmu
```

启动服务器：

```bash
node server.js
```

看到下面输出后，在浏览器打开页面：

```text
打开 http://localhost:3000
```

访问地址：

```text
http://localhost:3000
```


## 使用其他端口

默认端口是 `3000`。如果需要换端口，例如 `3001`：

```bash
cd /home/rc4/r2_pingmu
PORT=3001 node server.js
```

然后访问：

```text
http://localhost:3001
```

## 保存结果

页面点击确认按钮后，会向服务器发送保存请求。服务器会把当前队伍和 4 x 3 矩阵写入：

```text
/home/rc4/r2_pingmu/merlin-kfs-distribution.json
```

也可以在终端查看当前保存内容：

```bash
cat /home/rc4/r2_pingmu/merlin-kfs-distribution.json
```

const http = require("http");
const fs = require("fs");
const path = require("path");

const port = Number(process.env.PORT) || 3000;
const rootDir = __dirname;
const outputFile = path.join(rootDir, "merlin-kfs-distribution.json");
const currentScript = fs.realpathSync(__filename);

const mimeTypes = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".png": "image/png"
};

function send(response, statusCode, body, contentType = "text/plain; charset=utf-8") {
  response.writeHead(statusCode, {
    "Content-Type": contentType
  });
  response.end(body);
}

function sleep(milliseconds) {
  return new Promise((resolve) => {
    setTimeout(resolve, milliseconds);
  });
}

function isRunning(pid) {
  try {
    process.kill(pid, 0);
    return true;
  } catch (error) {
    return error.code === "EPERM";
  }
}

function getProcessInfo(pid) {
  try {
    const cmdline = fs.readFileSync(`/proc/${pid}/cmdline`, "utf8").split("\0").filter(Boolean);
    const cwd = fs.realpathSync(`/proc/${pid}/cwd`);
    const exe = fs.realpathSync(`/proc/${pid}/exe`);

    return { cmdline, cwd, exe };
  } catch (error) {
    return null;
  }
}

function resolveProcessArg(arg, cwd) {
  if (!arg || arg.startsWith("-")) {
    return null;
  }

  const candidate = path.isAbsolute(arg) ? arg : path.join(cwd, arg);

  try {
    return fs.realpathSync(candidate);
  } catch (error) {
    return path.resolve(cwd, arg);
  }
}

function isPreviousServer(pid) {
  if (pid === process.pid) {
    return false;
  }

  const processInfo = getProcessInfo(pid);
  if (!processInfo || path.basename(processInfo.exe) !== "node") {
    return false;
  }

  return processInfo.cmdline.some((arg, index) => {
    return index > 0 && resolveProcessArg(arg, processInfo.cwd) === currentScript;
  });
}

function findPreviousServers() {
  try {
    return fs.readdirSync("/proc")
      .filter((entry) => /^\d+$/.test(entry))
      .map(Number)
      .filter(isPreviousServer);
  } catch (error) {
    return [];
  }
}

async function stopPreviousServers() {
  const pids = findPreviousServers();

  if (pids.length === 0) {
    return;
  }

  console.log(`关闭前一次服务器: ${pids.join(", ")}`);

  for (const pid of pids) {
    try {
      process.kill(pid, "SIGTERM");
    } catch (error) {
      if (error.code !== "ESRCH") {
        console.warn(`无法关闭进程 ${pid}: ${error.message}`);
      }
    }
  }

  const deadline = Date.now() + 1500;
  while (Date.now() < deadline && pids.some(isRunning)) {
    await sleep(50);
  }

  for (const pid of pids.filter(isRunning)) {
    try {
      console.log(`强制关闭前一次服务器: ${pid}`);
      process.kill(pid, "SIGKILL");
    } catch (error) {
      if (error.code !== "ESRCH") {
        console.warn(`无法强制关闭进程 ${pid}: ${error.message}`);
      }
    }
  }
}

function serveFile(response, requestUrl) {
  const pathname = decodeURIComponent(new URL(requestUrl, `http://localhost:${port}`).pathname);
  const requestedPath = pathname === "/" ? "/index.html" : pathname;
  const filePath = path.normalize(path.join(rootDir, requestedPath));

  if (!filePath.startsWith(rootDir)) {
    send(response, 403, "Forbidden");
    return;
  }

  fs.readFile(filePath, (error, data) => {
    if (error) {
      send(response, 404, "Not found");
      return;
    }

    const contentType = mimeTypes[path.extname(filePath).toLowerCase()] || "application/octet-stream";
    send(response, 200, data, contentType);
  });
}

function saveDistribution(request, response) {
  let body = "";

  request.on("data", (chunk) => {
    body += chunk;
    if (body.length > 100_000) {
      request.destroy();
    }
  });

  request.on("end", () => {
    try {
      const payload = JSON.parse(body);
      const team = payload.team === "blue" ? "blue" : "red";
      const matrix = payload.matrix;

      if (!Array.isArray(matrix) || matrix.length !== 4 || matrix.some((row) => !Array.isArray(row) || row.length !== 3)) {
        send(response, 400, JSON.stringify({ ok: false, error: "Invalid matrix" }), "application/json; charset=utf-8");
        return;
      }

      const data = {
        team,
        matrix
      };

      fs.writeFileSync(outputFile, `${JSON.stringify(data, null, 2)}\n`, "utf8");
      console.log(`team ${team}`);
      console.log(matrix.map((row) => row.join(" ")).join("\n"));
      send(response, 200, JSON.stringify({ ok: true, file: path.basename(outputFile) }), "application/json; charset=utf-8");
    } catch (error) {
      send(response, 400, JSON.stringify({ ok: false, error: error.message }), "application/json; charset=utf-8");
    }
  });
}

const server = http.createServer((request, response) => {
  if (request.method === "POST" && request.url === "/save-distribution") {
    saveDistribution(request, response);
    return;
  }

  if (request.method === "GET") {
    serveFile(response, request.url);
    return;
  }

  send(response, 405, "Method not allowed");
});

server.on("error", (error) => {
  if (error.code === "EADDRINUSE") {
    console.error(`端口 ${port} 已被占用，且没有找到可关闭的前一次 server.js 进程。`);
  } else {
    console.error(error);
  }

  process.exit(1);
});

stopPreviousServers()
  .then(() => {
    server.listen(port, () => {
      console.log(`打开 http://localhost:${port}`);
    });
  })
  .catch((error) => {
    console.error(error);
    process.exit(1);
  });

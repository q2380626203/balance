#pragma once

const char html_debug_page[] = R"html(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32平衡车 - 调试面板</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #2c3e50 0%, #34495e 100%);
            min-height: 100vh;
            padding: 20px;
        }
        
        .header {
            text-align: center;
            color: white;
            margin-bottom: 30px;
        }
        
        .header h1 {
            font-size: 28px;
            margin-bottom: 10px;
        }
        
        .header p {
            opacity: 0.8;
            font-size: 16px;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
        }
        
        .panel {
            background: rgba(255, 255, 255, 0.95);
            border-radius: 12px;
            padding: 25px;
            box-shadow: 0 15px 35px rgba(0, 0, 0, 0.1);
            backdrop-filter: blur(10px);
        }
        
        .panel h2 {
            color: #2c3e50;
            margin-bottom: 20px;
            font-size: 22px;
            font-weight: 600;
            border-bottom: 2px solid #e74c3c;
            padding-bottom: 10px;
        }
        
        .error-controls {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
            margin-bottom: 25px;
        }
        
        .query-btn {
            background: linear-gradient(135deg, #e74c3c 0%, #c0392b 100%);
            color: white;
            border: none;
            border-radius: 8px;
            padding: 12px 20px;
            font-size: 14px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
            box-shadow: 0 4px 15px rgba(231, 76, 60, 0.3);
        }
        
        .query-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 25px rgba(231, 76, 60, 0.4);
        }
        
        .query-btn:active {
            transform: translateY(0);
        }
        
        .query-all-btn {
            grid-column: 1 / -1;
            background: linear-gradient(135deg, #3498db 0%, #2980b9 100%);
            box-shadow: 0 4px 15px rgba(52, 152, 219, 0.3);
        }
        
        .query-all-btn:hover {
            box-shadow: 0 8px 25px rgba(52, 152, 219, 0.4);
        }
        
        .error-display {
            background: #f8f9fa;
            border-radius: 8px;
            padding: 20px;
            border-left: 4px solid #e74c3c;
            min-height: 200px;
            max-height: 400px;
            overflow-y: auto;
        }
        
        .error-item {
            background: white;
            border-radius: 6px;
            padding: 15px;
            margin-bottom: 15px;
            border-left: 4px solid #e74c3c;
            box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
        }
        
        .error-item.no-error {
            border-left-color: #27ae60;
            background: #d5f4e6;
        }
        
        .error-type {
            font-weight: bold;
            color: #2c3e50;
            font-size: 16px;
            margin-bottom: 8px;
        }
        
        .error-code {
            font-family: 'Courier New', monospace;
            background: #ecf0f1;
            padding: 4px 8px;
            border-radius: 4px;
            display: inline-block;
            margin-right: 10px;
            font-size: 12px;
        }
        
        .error-desc {
            color: #7f8c8d;
            font-size: 14px;
            margin-top: 8px;
        }
        
        .loading {
            text-align: center;
            color: #7f8c8d;
            font-style: italic;
            padding: 40px;
        }
        
        .timestamp {
            color: #95a5a6;
            font-size: 12px;
            text-align: right;
            margin-top: 10px;
        }
        
        .nav-links {
            text-align: center;
            margin-top: 30px;
        }
        
        .nav-links a {
            color: white;
            text-decoration: none;
            background: rgba(255, 255, 255, 0.2);
            padding: 10px 20px;
            border-radius: 20px;
            margin: 0 10px;
            transition: all 0.3s ease;
        }
        
        .nav-links a:hover {
            background: rgba(255, 255, 255, 0.3);
            transform: translateY(-2px);
        }
        
        @media (max-width: 768px) {
            .container {
                grid-template-columns: 1fr;
                gap: 15px;
            }
            
            .error-controls {
                grid-template-columns: 1fr;
            }
            
            .header h1 {
                font-size: 24px;
            }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🔧 调试面板</h1>
        <p>电机错误诊断与监控</p>
    </div>
    
    <div class="container">
        <!-- 错误查询控制面板 -->
        <div class="panel">
            <h2>📊 错误查询</h2>
            <div class="error-controls">
                <button class="query-btn" onclick="queryError(0)">🔧 电机异常</button>
                <button class="query-btn" onclick="queryError(1)">📏 编码器异常</button>
                <button class="query-btn" onclick="queryError(3)">🎛️ 控制器异常</button>
                <button class="query-btn" onclick="queryError(4)">⚙️ 系统异常</button>
                <button class="query-btn" onclick="clearErrors()" style="background: linear-gradient(135deg, #f44336 0%, #c62828 100%);">🔧 清除错误</button>
                <button class="query-btn" onclick="enableMotor()" style="background: linear-gradient(135deg, #4caf50 0%, #388e3c 100%);">🔋 使能电机</button>
                <button class="query-btn" onclick="restartMotor()" style="background: linear-gradient(135deg, #ff9800 0%, #f57c00 100%);">🔄 重启电机</button>
                <button class="query-btn" onclick="startBalance()" style="background: linear-gradient(135deg, #2196f3 0%, #1976d2 100%);">▶️ 开启自平衡</button>
                <button class="query-btn" onclick="stopBalance()" style="background: linear-gradient(135deg, #607d8b 0%, #455a64 100%);">⏸️ 关闭自平衡</button>
            </div>
            <div class="timestamp" id="lastQuery">最后查询时间: 未查询</div>
        </div>
        
        <!-- 错误显示面板 -->
        <div class="panel">
            <h2>📋 错误状态</h2>
            <div class="error-display" id="errorDisplay">
                <div class="loading">点击查询按钮开始诊断...</div>
            </div>
        </div>
    </div>
    
    <div class="nav-links">
        <a href="/">🏠 返回主页</a>
        <a href="/test">🔗 WebSocket测试</a>
    </div>
    
    <script>
        let currentErrors = {};
        
        function updateTimestamp() {
            const now = new Date();
            document.getElementById('lastQuery').textContent = 
                '最后查询时间: ' + now.toLocaleTimeString();
        }
        
        function showLoading() {
            document.getElementById('errorDisplay').innerHTML = 
                '<div class="loading">🔄 正在查询错误状态...</div>';
        }
        
        function queryError(type) {
            showLoading();
            updateTimestamp();
            
            fetch('/api/debug/query-error', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    error_type: type
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    // 等待一下让查询完成，然后获取错误状态
                    setTimeout(() => {
                        getErrorStatus();
                    }, 500);
                } else {
                    showError('查询失败: ' + (data.message || '未知错误'));
                }
            })
            .catch(error => {
                showError('网络错误: ' + error.message);
            });
        }
        
        function clearErrors() {
            showLoading();
            updateTimestamp();
            
            fetch('/api/debug/clear-errors', {
                method: 'POST'
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    // 清除错误后，重新获取错误状态
                    setTimeout(() => {
                        getErrorStatus();
                    }, 500);
                } else {
                    showError('清除错误失败: ' + (data.message || '未知错误'));
                }
            })
            .catch(error => {
                showError('网络错误: ' + error.message);
            });
        }
        
        function enableMotor() {
            showLoading();
            updateTimestamp();
            
            fetch('/api/debug/enable-motor', {
                method: 'POST'
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    // 使能电机后，重新获取错误状态
                    setTimeout(() => {
                        getErrorStatus();
                    }, 500);
                } else {
                    showError('使能电机失败: ' + (data.message || '未知错误'));
                }
            })
            .catch(error => {
                showError('网络错误: ' + error.message);
            });
        }
        
        function restartMotor() {
            showLoading();
            updateTimestamp();
            
            fetch('/api/debug/restart-motor', {
                method: 'POST'
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    // 重启电机后，重新获取错误状态
                    setTimeout(() => {
                        getErrorStatus();
                    }, 1000); // 重启需要更长时间
                } else {
                    showError('重启电机失败: ' + (data.message || '未知错误'));
                }
            })
            .catch(error => {
                showError('网络错误: ' + error.message);
            });
        }
        
        function startBalance() {
            showLoading();
            updateTimestamp();
            
            fetch('/api/debug/start-balance', {
                method: 'POST'
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showMessage('✅ 自平衡已启动');
                    setTimeout(() => {
                        getErrorStatus();
                    }, 500);
                } else {
                    showError('启动自平衡失败: ' + (data.message || '未知错误'));
                }
            })
            .catch(error => {
                showError('网络错误: ' + error.message);
            });
        }
        
        function stopBalance() {
            showLoading();
            updateTimestamp();
            
            fetch('/api/debug/stop-balance', {
                method: 'POST'
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showMessage('⏸️ 自平衡已停止');
                    setTimeout(() => {
                        getErrorStatus();
                    }, 500);
                } else {
                    showError('停止自平衡失败: ' + (data.message || '未知错误'));
                }
            })
            .catch(error => {
                showError('网络错误: ' + error.message);
            });
        }
        
        function getErrorStatus() {
            fetch('/api/debug/error-status')
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    displayErrors(data.errors);
                } else {
                    showError('获取错误状态失败: ' + (data.message || '未知错误'));
                }
            })
            .catch(error => {
                showError('网络错误: ' + error.message);
            });
        }
        
        function displayErrors(errors) {
            const display = document.getElementById('errorDisplay');
            let html = '';
            
            const errorTypes = [
                { key: 'motor_error', name: '🔧 电机异常', desc_key: 'motor_error_desc' },
                { key: 'encoder_error', name: '📏 编码器异常', desc_key: 'encoder_error_desc' },
                { key: 'controller_error', name: '🎛️ 控制器异常', desc_key: 'controller_error_desc' },
                { key: 'system_error', name: '⚙️ 系统异常', desc_key: 'system_error_desc' }
            ];
            
            let hasAnyError = false;
            
            errorTypes.forEach(type => {
                const errorCode = errors[type.key];
                const errorDesc = errors[type.desc_key] || '正常';
                const hasError = errorCode && errorCode !== 0;
                
                if (hasError) {
                    hasAnyError = true;
                }
                
                html += `<div class="error-item ${hasError ? '' : 'no-error'}">
                    <div class="error-type">${type.name}</div>
                    <div>
                        <span class="error-code">0x${errorCode.toString(16).toUpperCase().padStart(8, '0')}</span>
                        <div class="error-desc">
                            ${hasError ? errorDesc : '✅ 正常'}
                        </div>
                    </div>
                </div>`;
            });
            
            if (!hasAnyError) {
                html = `<div class="error-item no-error">
                    <div class="error-type">✅ 系统状态</div>
                    <div class="error-desc">所有系统运行正常，未检测到异常</div>
                </div>` + html;
            }
            
            display.innerHTML = html;
        }
        
        function getErrorDescription(errorCode, errorType) {
            // 这里可以扩展错误码的详细描述
            if (errorCode === 0) return '正常';
            return `检测到异常 (错误码: ${errorCode})`;
        }
        
        function showError(message) {
            document.getElementById('errorDisplay').innerHTML = `
                <div class="error-item">
                    <div class="error-type">❌ 系统错误</div>
                    <div class="error-desc">${message}</div>
                </div>
            `;
        }
        
        function showMessage(message) {
            document.getElementById('errorDisplay').innerHTML = `
                <div class="error-item no-error">
                    <div class="error-type">💡 系统信息</div>
                    <div class="error-desc">${message}</div>
                </div>
            `;
        }
        
        // 页面加载完成后自动查询一次状态
        window.addEventListener('load', function() {
            setTimeout(() => {
                getErrorStatus();
            }, 1000);
        });
    </script>
</body>
</html>
)html";
#pragma once

const char html_page_fixed[] = R"html(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32平衡车控制面板</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 10px;
        }
        
        .container {
            max-width: 1000px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }
        
        .panel {
            background: rgba(255, 255, 255, 0.95);
            border-radius: 12px;
            padding: 20px;
            box-shadow: 0 10px 25px rgba(0, 0, 0, 0.1);
            backdrop-filter: blur(10px);
        }
        
        .panel h2 {
            color: #2c3e50;
            margin-bottom: 15px;
            font-size: 20px;
            font-weight: 600;
            text-align: center;
        }
        
        .status-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
            margin-bottom: 20px;
        }
        
        .status-card {
            background: #f8f9fa;
            border-radius: 8px;
            padding: 12px;
            text-align: center;
            border: 2px solid transparent;
            transition: all 0.3s ease;
        }
        
        .status-card.connected {
            border-color: #10b981;
            background: rgba(16, 185, 129, 0.1);
        }
        
        .status-card.disconnected {
            border-color: #ef4444;
            background: rgba(239, 68, 68, 0.1);
        }
        
        .status-card.balanced {
            border-color: #3b82f6;
            background: rgba(59, 130, 246, 0.1);
        }
        
        .status-card.unbalanced {
            border-color: #f59e0b;
            background: rgba(245, 158, 11, 0.1);
        }
        
        .status-label {
            font-size: 12px;
            color: #6b7280;
            margin-bottom: 4px;
            font-weight: 500;
        }
        
        .status-value {
            font-size: 14px;
            font-weight: bold;
            color: #1f2937;
        }
        
        .angle-display {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin: 15px 0;
        }
        
        .angle-card {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
            border-radius: 8px;
            padding: 12px;
            text-align: center;
        }
        
        .angle-label {
            font-size: 12px;
            opacity: 0.9;
            margin-bottom: 4px;
        }
        
        .angle-value {
            font-size: 20px;
            font-weight: bold;
            margin-bottom: 2px;
        }
        
        .angle-unit {
            font-size: 10px;
            opacity: 0.8;
        }
        
        .config-form {
            display: grid;
            gap: 12px;
        }
        
        .form-group {
            display: flex;
            flex-direction: column;
        }
        
        .form-group label {
            font-size: 12px;
            color: #374151;
            margin-bottom: 4px;
            font-weight: 500;
        }
        
        .form-group input[type="number"] {
            padding: 8px 12px;
            border: 2px solid #e5e7eb;
            border-radius: 6px;
            font-size: 14px;
            transition: border-color 0.3s ease;
        }
        
        .form-group input[type="number"]:focus {
            outline: none;
            border-color: #667eea;
        }
        
        .form-group input[type="checkbox"] {
            width: 16px;
            height: 16px;
            margin-right: 8px;
        }
        
        .checkbox-group {
            display: flex;
            align-items: center;
            margin: 8px 0;
        }
        
        .button-group {
            display: flex;
            gap: 10px;
            margin-top: 15px;
        }
        
        .btn {
            flex: 1;
            padding: 10px 16px;
            border: none;
            border-radius: 6px;
            font-size: 14px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
        }
        
        .btn-primary:hover {
            transform: translateY(-2px);
            box-shadow: 0 10px 20px rgba(102, 126, 234, 0.3);
        }
        
        .btn-secondary {
            background: #f3f4f6;
            color: #374151;
            border: 2px solid #e5e7eb;
        }
        
        .btn-secondary:hover {
            background: #e5e7eb;
            transform: translateY(-2px);
        }
        
        .connection-status {
            position: fixed;
            top: 10px;
            right: 10px;
            padding: 6px 12px;
            border-radius: 20px;
            color: white;
            font-weight: 600;
            font-size: 12px;
            z-index: 1000;
        }
        
        .connection-status.connected {
            background: #10b981;
        }
        
        .connection-status.disconnected {
            background: #ef4444;
        }
        
        .connection-status.connecting {
            background: #f59e0b;
        }
        
        @media (max-width: 768px) {
            .container {
                grid-template-columns: 1fr;
                gap: 12px;
            }
            
            .panel {
                padding: 15px;
            }
            
            .status-grid {
                grid-template-columns: 1fr;
                gap: 8px;
            }
            
            .angle-display {
                grid-template-columns: repeat(3, 1fr);
                gap: 8px;
            }
            
            .button-group {
                gap: 8px;
            }
        }
    </style>
</head>
<body>
    <div class="connection-status" id="connectionStatus">连接中...</div>
    
    <div class="container">
        <div class="panel">
            <h2>实时监控面板</h2>
            
            <div class="status-grid">
                <div class="status-card" id="bleStatus">
                    <div class="status-label">BLE连接</div>
                    <div class="status-value" id="bleValue">断开</div>
                </div>
                <div class="status-card" id="balanceStatus">
                    <div class="status-label">平衡状态</div>
                    <div class="status-value" id="balanceValue">未平衡</div>
                </div>
            </div>
            
            <div class="angle-display">
                <div class="angle-card">
                    <div class="angle-label">Roll</div>
                    <div class="angle-value" id="rollValue">0.0</div>
                    <div class="angle-unit">°</div>
                </div>
                <div class="angle-card">
                    <div class="angle-label">Pitch</div>
                    <div class="angle-value" id="pitchValue">0.0</div>
                    <div class="angle-unit">°</div>
                </div>
                <div class="angle-card">
                    <div class="angle-label">Yaw</div>
                    <div class="angle-value" id="yawValue">0.0</div>
                    <div class="angle-unit">°</div>
                </div>
            </div>
        </div>
        
        <div class="panel">
            <h2>参数调整面板</h2>
            
            <div class="config-form">
                <div class="form-group">
                    <label for="targetPitch">目标俯仰角 (°)</label>
                    <input type="number" id="targetPitch" step="0.1" min="-45" max="45">
                </div>
                
                <div class="form-group">
                    <label for="pitchTolerance">俯仰容差 (°)</label>
                    <input type="number" id="pitchTolerance" step="0.1" min="0" max="10">
                </div>
                
                <div class="form-group">
                    <label for="motorSpeed">电机固定速度</label>
                    <input type="number" id="motorSpeed" step="1" min="0" max="100">
                </div>
                
                <div class="form-group">
                    <label for="enableDelay">启动延迟 (ms)</label>
                    <input type="number" id="enableDelay" step="100" min="0" max="5000">
                </div>
                
                <div class="form-group">
                    <label for="restartThreshold">重启阈值 (°)</label>
                    <input type="number" id="restartThreshold" step="0.1" min="0" max="90">
                </div>
                
                <div class="checkbox-group">
                    <input type="checkbox" id="autoRestart">
                    <label for="autoRestart">启用自动重启</label>
                </div>
                
                <div class="button-group">
                    <button class="btn btn-primary" onclick="saveConfig()">保存设置</button>
                    <button class="btn btn-secondary" onclick="resetConfig()">恢复默认</button>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        let ws = null;
        let reconnectInterval = null;
        
        function updateConnectionStatus(status) {
            const statusEl = document.getElementById('connectionStatus');
            statusEl.className = 'connection-status ' + status;
            statusEl.textContent = status === 'connected' ? '已连接' : 
                                  status === 'connecting' ? '连接中...' : '已断开';
        }
        
        function connectWebSocket() {
            try {
                ws = new WebSocket('ws://' + window.location.host + '/ws');
                
                ws.onopen = function() {
                    console.log('WebSocket连接成功');
                    updateConnectionStatus('connected');
                    if (reconnectInterval) {
                        clearInterval(reconnectInterval);
                        reconnectInterval = null;
                    }
                };
                
                ws.onclose = function() {
                    console.log('WebSocket连接关闭');
                    updateConnectionStatus('disconnected');
                    startReconnect();
                };
                
                ws.onerror = function(error) {
                    console.error('WebSocket错误:', error);
                    updateConnectionStatus('disconnected');
                };
                
                ws.onmessage = function(event) {
                    try {
                        const data = JSON.parse(event.data);
                        updateUI(data);
                    } catch (e) {
                        console.error('解析数据错误:', e);
                    }
                };
            } catch (error) {
                console.error('WebSocket连接失败:', error);
                updateConnectionStatus('disconnected');
                startReconnect();
            }
        }
        
        function startReconnect() {
            if (!reconnectInterval) {
                reconnectInterval = setInterval(() => {
                    console.log('尝试重新连接...');
                    updateConnectionStatus('connecting');
                    connectWebSocket();
                }, 3000);
            }
        }
        
        function updateUI(data) {
            // 更新连接状态
            const bleStatus = document.getElementById('bleStatus');
            const bleValue = document.getElementById('bleValue');
            if (data.ble_connected) {
                bleStatus.className = 'status-card connected';
                bleValue.textContent = '已连接';
            } else {
                bleStatus.className = 'status-card disconnected';
                bleValue.textContent = '断开';
            }
            
            
            // 更新平衡状态
            const balanceStatus = document.getElementById('balanceStatus');
            const balanceValue = document.getElementById('balanceValue');
            if (data.in_tolerance) {
                balanceStatus.className = 'status-card balanced';
                balanceValue.textContent = '平衡';
            } else {
                balanceStatus.className = 'status-card unbalanced';
                balanceValue.textContent = '未平衡';
            }
            
            // 更新角度数据
            document.getElementById('rollValue').textContent = data.roll.toFixed(1);
            document.getElementById('pitchValue').textContent = data.pitch.toFixed(1);
            document.getElementById('yawValue').textContent = data.yaw.toFixed(1);
            
        }
        
        async function loadConfig() {
            try {
                const response = await fetch('/api/config');
                if (response.ok) {
                    const config = await response.json();
                    document.getElementById('targetPitch').value = config.target_pitch_angle;
                    document.getElementById('pitchTolerance').value = config.pitch_tolerance;
                    document.getElementById('motorSpeed').value = config.motor_fixed_speed;
                    document.getElementById('enableDelay').value = config.enable_delay_ms;
                    document.getElementById('restartThreshold').value = config.restart_threshold;
                    document.getElementById('autoRestart').checked = config.auto_restart_enabled;
                }
            } catch (error) {
                console.error('加载配置失败:', error);
                alert('加载配置失败');
            }
        }
        
        async function saveConfig() {
            const config = {
                target_pitch_angle: parseFloat(document.getElementById('targetPitch').value),
                pitch_tolerance: parseFloat(document.getElementById('pitchTolerance').value),
                motor_fixed_speed: parseFloat(document.getElementById('motorSpeed').value),
                enable_delay_ms: parseInt(document.getElementById('enableDelay').value),
                restart_threshold: parseFloat(document.getElementById('restartThreshold').value),
                auto_restart_enabled: document.getElementById('autoRestart').checked
            };
            
            try {
                const response = await fetch('/api/config', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify(config)
                });
                
                if (response.ok) {
                    const result = await response.json();
                    if (result.success) {
                        alert('配置保存成功');
                    } else {
                        alert('配置保存失败');
                    }
                } else {
                    alert('配置保存失败');
                }
            } catch (error) {
                console.error('保存配置失败:', error);
                alert('保存配置失败');
            }
        }
        
        async function resetConfig() {
            if (confirm('确定要恢复默认设置吗？')) {
                try {
                    const response = await fetch('/api/config/reset', {
                        method: 'POST'
                    });
                    
                    if (response.ok) {
                        const result = await response.json();
                        if (result.success) {
                            alert('已恢复默认设置');
                            loadConfig();
                        } else {
                            alert('恢复默认设置失败');
                        }
                    } else {
                        alert('恢复默认设置失败');
                    }
                } catch (error) {
                    console.error('恢复默认设置失败:', error);
                    alert('恢复默认设置失败');
                }
            }
        }
        
        // 页面加载完成后初始化
        document.addEventListener('DOMContentLoaded', function() {
            loadConfig();
            connectWebSocket();
        });
    </script>
</body>
</html>
)html";
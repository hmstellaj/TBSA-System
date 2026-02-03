/* script.js - 서버 생성 실시간 경로 이미지 방식 */
let currentSeq = 1;
let lastLogMsg = "";
let lastImageUpdate = 0;
let lastPathImageUpdate = 0;

let staticPathTimestamp = Date.now();
let lastPathNodeCount = 0;

let isImageLoading = false;

// SEQ 2 상태 표시용 변수
let lastActionStatus = "";

// 실시간 경로 이미지 업데이트 간격 (ms)
const PATH_IMAGE_INTERVAL = 200;  // 0.5초마다 업데이트

window.addEventListener('load', () => {
    console.log('페이지 로드 완료 - 서버 생성 실시간 경로 이미지 모드');
    gameLoop();
});

function selectSeq(seq) {
    fetch('/change_seq', { 
        method: 'POST', 
        headers: { 'Content-Type': 'application/json' }, 
        body: JSON.stringify({ seq: seq }) 
    })
    .then(r => r.json())
    .then(data => { 
        if (data.status === 'OK') { 
            currentSeq = seq; 
            refresh(); 
        } 
    });
}

function setQuickDest(x, z) {
    document.getElementById('dest-input').value = `${x}, ${z}`;
    setDestination();
}

function setDestination() {
    const input = document.getElementById('dest-input').value.trim();
    const status = document.getElementById('dest-status');
    const coords = input.replace(/[()]/g, '').split(',').map(s => parseFloat(s.trim()));
    
    if (coords.length !== 2 || coords.some(isNaN)) { 
        status.textContent = '❌ 형식 오류'; 
        return; 
    }
    
    fetch('/set_destination', {
        method: 'POST', 
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ destination: `${coords[0]},0,${coords[1]}` })
    })
    .then(r => r.json())
    .then(data => { 
        status.textContent = data.status === 'OK' ? `✅ (${coords[0]}, ${coords[1]})` : '❌ 실패'; 
        // 목적지 설정 후 즉시 이미지 갱신
        if (data.status === 'OK') {
            updateRealtimePathImage();
        }
    });
}

// 전투 액션 전송 (FIRE, RESCAN, RETREAT)
function sendCombatAction(action) {
    fetch('/combat_action', { 
        method: 'POST', 
        headers: { 'Content-Type': 'application/json' }, 
        body: JSON.stringify({ action: action }) 
    });
}

// 타겟 재탐색
function handleRescan() {
    sendCombatAction('RESCAN');
}

// SCAN 탐색 방향 설정 함수
function setScanDir(dir) {
    fetch('/set_scan_direction', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ direction: dir })
    })
    .then(res => res.json())
    .then(data => { if (data.status === 'OK') refresh(); })
    .catch(err => console.error("방향 설정 실패:", err));
}

// 전투 모드 변경 요청 함수
async function setSeq2Mode(mode) {
    try {
        const res = await fetch("/set_seq2_mode", {
            method: "POST",
            headers: {"Content-Type":"application/json"},
            body: JSON.stringify({ mode })
        });
        const j = await res.json();
        console.log("서버 모드 전환 응답:", j);
    } catch (e) {
        console.error("모드 전환 실패:", e);
    }
}

// 실시간 경로 이미지 업데이트 함수
function updateRealtimePathImage() {
    if (isImageLoading) return;

    const imgElement = document.getElementById('realtimePathImage');
    if (!imgElement) return;

    const now = Date.now();
    
    // 업데이트 간격 체크
    if (now - lastPathImageUpdate < 100) return;

    // 컨테이너 크기 계산
    const container = imgElement.parentElement;
    let width = container ? container.clientWidth : 640;
    let height = container ? container.clientHeight : 640;
    width = Math.max(400, Math.min(1000, width));
    height = Math.max(400, Math.min(1000, height));

    // 3. 로딩 시작 표시
    isImageLoading = true; 

    // 4. 새 이미지 객체를 미리 만들어서 로딩함 (깜빡임 방지 테크닉)
    const newImg = new Image();
    
    newImg.onload = () => {
        // 로딩이 성공하면 실제 화면에 반영
        imgElement.src = newImg.src;
        lastPathImageUpdate = Date.now();
        isImageLoading = false; // 락 해제
        
        // 다음 프레임을 위해 즉시 재호출 (애니메이션처럼 부드럽게)
        // 상황에 따라 requestAnimationFrame을 써도 됨
        setTimeout(updateRealtimePathImage, 50); 
    };

    newImg.onerror = () => {
        console.error("이미지 로드 실패, 재시도");
        isImageLoading = false; // 실패해도 락 해제
        setTimeout(updateRealtimePathImage, 500); // 0.5초 뒤 재시도
    };

    // 요청 시작
    newImg.src = `/realtime_path_image?width=${width}&height=${height}&t=${now}`;
}

function refresh() {
    const t = new Date().getTime();
    
    return fetch('/debug_status')
    .then(r => r.json())
    .then(j => {
        // UI 상태 업데이트
        const banner = document.getElementById('msg-banner');
        banner.textContent = j.msg || "CONNECTED";
        const serverSeq = j.seq;
        
        document.querySelectorAll('.layout-content').forEach(l => l.classList.remove('active'));
        
        // SEQ 상태 표시 업데이트
        const seqDisplay = document.getElementById('current-seq-display');
        if (seqDisplay) {
            seqDisplay.textContent = `SEQ ${serverSeq}`;
            seqDisplay.className = `seq-display seq-${serverSeq}`;
        }
        
        const combatModeDisplay = document.getElementById('combat-mode-display');
        const combatModeBadge = document.getElementById('combat-mode-badge');

        if (serverSeq === 2) {
            combatModeDisplay.style.display = 'flex';
            const mode = j.combat_mode || 'SCAN';
            combatModeBadge.textContent = mode;
            combatModeBadge.className = 'mode-badge mode-' + mode.toLowerCase();
        } else {
            combatModeDisplay.style.display = 'none';
        }
        
        document.getElementById('position-panel').classList.toggle('hidden', serverSeq === 2);
        document.getElementById('destination-input').classList.toggle('active', serverSeq !== 2);

        // ═══════════════════════════════════════════════════════════════
        // SEQ 1, 3: 서버 생성 실시간 경로 이미지
        // ═══════════════════════════════════════════════════════════════
        if (serverSeq === 1 || serverSeq === 3) {
            document.getElementById('navigation-layout').classList.add('active');

            // 실시간 경로 이미지 업데이트
            updateRealtimePathImage();

            const currentNodes = j.path_nodes || 0;
            if (currentNodes !== lastPathNodeCount && currentNodes > 0) {
                console.log(`📡 경로 변경 감지! (${lastPathNodeCount} -> ${currentNodes}) 이미지 새로고침`);
                staticPathTimestamp = new Date().getTime(); // 타임스탬프 갱신으로 강제 새로고침 트리거
                lastPathNodeCount = currentNodes;
            }
            
            const pipImg = document.getElementById('staticPathPip');
            if (pipImg) {
                // 현재 SEQ에 맞는 정적 경로 이미지 URL 설정
                // app.py의 /get_static_path/<seq> 엔드포인트 활용
                const targetSrc = `/get_static_path/${serverSeq}?t=${staticPathTimestamp}`;
                
                // src가 바뀌었을 때만 업데이트 (깜빡임 방지)
                if (!pipImg.src.endsWith(targetSrc) && pipImg.getAttribute('src') !== targetSrc) {
                    pipImg.src = targetSrc;
                }
                pipImg.style.display = 'block';
            }

            // 경로 노드 정보 업데이트
            const currentNodeIdx = j.current_node !== undefined ? j.current_node : '-';
            const totalNodes = j.path_nodes !== undefined ? j.path_nodes : '-';
            document.getElementById('path-node-info').textContent = `${currentNodeIdx}/${totalNodes}`;

            if (j.path_nodes && j.current_node) {
                document.getElementById('path-node-info').textContent = `${j.current_node}/${j.path_nodes}`;
            } else if (j.path_nodes) {
                document.getElementById('path-node-info').textContent = `-/${j.path_nodes}`;
            } else {
                document.getElementById('path-node-info').textContent = '-/-';
            }

            // 로그 업데이트
            if (j.log && j.log !== lastLogMsg) {
                const logArea = document.getElementById('driving-log');
                logArea.innerHTML = `[${new Date().toLocaleTimeString()}] ${j.log}\n` + logArea.innerHTML;
                lastLogMsg = j.log;
            }
        } 
        // ═══════════════════════════════════════════════════════════════
        // SEQ 2: 전투
        // ═══════════════════════════════════════════════════════════════
        else if (serverSeq === 2) {
            document.getElementById('combat-layout').classList.add('active');
            document.getElementById('combat-overlay').src = '/overlay/left?t=' + t;
            
            const combatMode = j.combat_mode || 'SCAN';
            
            // SCAN 방향 선택 버튼 제어 로직 추가
            const scanQBtn = document.getElementById('scan-q-btn');
            const scanEBtn = document.getElementById('scan-e-btn');
            const scanDirCard = document.getElementById('scan-direction-card');
            const fireReady = j.fire_ready || false;
            const lockedTarget = j.locked_target;
            const hasTarget = lockedTarget && lockedTarget.bbox;
            const autoAttack = j.auto_attack_active || false;
            
            // 서버에서 받은 타겟 목록 (SCAN 결과 + is_locked 플래그 포함)
            const targets = j.detected_targets || [];
            
            // 버튼 상태 업데이트
            const standbyBtn = document.getElementById('standby-btn');
            const rescanBtn = document.getElementById('rescan-btn');
            const retreatBtn = document.getElementById('retreat-btn');
            const fireBtn = document.getElementById('fire-btn');
            const actionStatus = document.getElementById('action-status-text');
            
            // SCAN 또는 RESCAN(서버에서는 결국 SCAN 모드)일 때만 활성화
            if (combatMode === 'SCAN') {
                scanQBtn.disabled = false;
                scanEBtn.disabled = false;
                scanDirCard.style.opacity = "1.0"; // 시각적으로 활성화 표시
            } else {
                scanQBtn.disabled = true;
                scanEBtn.disabled = true;
                scanDirCard.style.opacity = "0.5"; // 비활성화 시 흐리게 처리
            }

            // STANDBY 버튼: SCAN 모드일 때만 활성화
            if (combatMode === 'SCAN') {
                standbyBtn.disabled = false;
                standbyBtn.classList.remove('active-mode');
            } else {
                standbyBtn.disabled = true;
                standbyBtn.classList.add('active-mode');
            }
            
            // RESCAN, RETREAT 버튼: SCAN (적군 확인 되었을때)과 STANDBY 모드일 때 활성화
            const hasEnemies = targets.length > 0;
            const isCombatReady = ((combatMode === 'SCAN' && hasEnemies) || combatMode === 'STANDBY');
            rescanBtn.disabled = !isCombatReady;
            retreatBtn.disabled = !isCombatReady;         
       
            // 공격 버튼 활성화 로직 (단순화) (0130 추가)
            if (serverSeq === 2 && j.combat_mode === 'STANDBY') {
                fireBtn.disabled = false;
                fireBtn.classList.add('ready');
                fireBtn.textContent = "🔥 포격";  // 항상 고정
            } else {
                fireBtn.disabled = true;
                fireBtn.classList.remove('ready');
                fireBtn.textContent = "🔥 포격";  // 항상 고정
            }

            // 상태 텍스트 업데이트 로직 수정
            let newStatusText = "";
            let newStatusColor = "";

            if (combatMode === 'SCAN') {
                if (!j.scan_direction) {
                    newStatusText = '📡 방향(Q/E)을 선택하세요';
                    newStatusColor = '#2196F3';
                } else {
                    newStatusText = '🔍 객체 식별 중...';
                    newStatusColor = '#2196F3';
                }
            } else if (combatMode === 'STANDBY') {
                if (fireReady) {
                    newStatusText = '🎯 타겟 락온 완료 - FIRE 가능!';
                    newStatusColor = '#f44336';
                } else if (hasTarget) {
                    newStatusText = '⏳ 타겟 조준 중...';
                    newStatusColor = '#FF9800';
                } else {
                    newStatusText = '🔒 STANDBY 모드 - 타겟 대기 중...';
                    newStatusColor = '#4CAF50';
                }
            }

            // 텍스트가 변경되었을 때만 DOM 업데이트 실행
            if (newStatusText !== lastActionStatus) {
                actionStatus.textContent = newStatusText;
                actionStatus.style.color = newStatusColor;
                lastActionStatus = newStatusText; // 현재 상태 저장
            }

            // 타겟 카운트 업데이트
            document.getElementById('target-count').textContent = `(${targets.length})`;
            
            // 락된 타겟 정보 업데이트
            if (lockedTarget) {
                document.getElementById('lock-distance').textContent = 
                    lockedTarget.distance_m ? `${lockedTarget.distance_m.toFixed(1)}m` : '-';
                    // 전달받은 정밀 좌표(XYZ) 출력
                if (lockedTarget.position) {
                    const {x, y, z} = lockedTarget.position;
                    document.getElementById('lock-pos').textContent = `X:${x}, Y:${y}, Z:${z}`;
                } else {
                    document.getElementById('lock-pos').textContent = '-';
                }

                document.getElementById('lock-yaw').textContent = 
                    lockedTarget.yaw_error_deg !== undefined ? `${lockedTarget.yaw_error_deg.toFixed(1)}°` : '-';
                
                // 락된 타겟 카드 하이라이트
                document.getElementById('locked-target-card').style.borderColor = '#d16666';
            } else {
                document.getElementById('lock-distance').textContent = '-';
                document.getElementById('lock-yaw').textContent = '-';
                document.getElementById('locked-target-card').style.borderColor = '#333';
            }
            
            // ✅ 탐지된 타겟 리스트 업데이트 (서버에서 is_locked 플래그 사용)
            const targetList = document.getElementById('target-list');
            targetList.innerHTML = targets.slice(0, 10).map((t, i) => {
                const isLocked = t.is_locked || false;  // 서버에서 계산된 값 사용
                const dist = t.distance_m ? `${t.distance_m.toFixed(1)}m` : '';
                const className = t.className || t.category || 'Unknown';
                
                // ✅ locked 타겟은 'target-locked' 클래스 (빨간색)
                const itemClass = isLocked ? 'target-item target-locked' : 'target-item';
                const icon = isLocked ? '🔴' : '🔘';
                
                return `<div class="${itemClass}">
                    ${icon} ${className} ${dist}
                </div>`;
            }).join('');
        } 
        // ═══════════════════════════════════════════════════════════════
        // SEQ 4: 자율주행
        // ═══════════════════════════════════════════════════════════════
        else if (serverSeq === 4) {
            document.getElementById('autonomous-layout').classList.add('active');
            document.getElementById('autonomous-view').src = '/view_autonomous?t=' + t;
            document.getElementById('autonomous-costmap-global').src = '/view_autonomous?t=' + t;
        }

        // 공통 정보 업데이트
        if (j.tank_pose) document.getElementById('current-pos').textContent = `(${j.tank_pose[0].toFixed(1)}, ${j.tank_pose[1].toFixed(1)})`;
        if (j.destination) document.getElementById('destination-pos').textContent = `(${j.destination[0].toFixed(1)}, ${j.destination[1].toFixed(1)})`;
        document.getElementById('path-nodes').textContent = j.path_nodes ? `${j.path_nodes}개` : '0';
    })
    .catch(err => {
        console.error('디버그 상태 오류:', err);
    });
}

function gameLoop() {
    refresh().finally(() => {
        setTimeout(gameLoop, 150);
    });
}
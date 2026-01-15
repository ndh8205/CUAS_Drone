# Claude Code 설치 가이드 (Ubuntu 24.04)

## 1. Node.js 설치 (18 이상 필요)

```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs
```

## 2. Claude Code 설치

```bash
npm install -g @anthropic-ai/claude-code
```

## 3. 실행 및 인증

```bash
claude
```

처음 실행하면 브라우저가 열리면서 Anthropic 계정 로그인 화면이 나타남. 로그인하면 인증 완료.

## 4. 설치 확인

```bash
claude --version
```

## 사용법

프로젝트 폴더에서 `claude` 명령어로 실행. 터미널에서 코드 수정, 파일 생성, 명령어 실행 등 가능.

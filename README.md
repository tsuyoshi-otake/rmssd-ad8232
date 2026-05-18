# rmssd-ad8232

M5Stack Basic + AD8232 で心電図を取って、R波を検出し、RR間隔のRMSSDを心電図波形と一緒に表示するHRVビューア。副交感神経活動の proxy として、**自分のベースラインとの相対変化** を見るためのもの。

![ECG HRV viewer wiring and guide](ECG%20HRV%20viewer%20wiring%20and%20guide.png)

## ⚠️ 安全と免責

- **医療機器ではない。** 自己実験・教育目的に限る。診断や治療判断には絶対に使わない。
- **自分自身に対してのみ使う。** 他人、子供、ペースメーカー等の植え込み型医療機器を装着している人には使用しないこと。
- **バッテリー駆動を強く推奨。** PC の USB に繋いだまま電極を体に貼ると、PC グランド経由で漏れ電流が流れる可能性があり、AC 由来の 50/60Hz ノイズも盛大に乗る。書き込みが終わったら USB を抜き、M5Stack 内蔵バッテリーで運用する。
- **電極は使い捨て。** 使い回さない。

## 必要なもの

| 部品 | 備考 |
|------|------|
| M5Stack Basic | ESP32 + 320x240 LCD + MicroSDスロット内蔵 |
| AD8232 ECG モジュール | Keyestudio 版（本プロジェクトの配線表はこれ前提）。SparkFun 版はピン名が `SDN`/`3.3V` などで少し違うので適宜読み替え |
| 使い捨て ECG 電極 × 3 | スナップ式（バイオセンスやアンブー製のホルター用ゲル電極）。 |
| 電極リード線 | AD8232 付属のスナップ・モノラルジャック3本（赤=RA, 黄=LA, 緑/黒=RL） |
| ジャンパーワイヤー数本 | M5Stack BASE:AAA に挿す用 |
| MicroSDカード | 長期ログを取るなら必須。FAT32フォーマット推奨。容量は1GBで十分（数日分のログでも数十MB程度） |

## 配線 (M5Stack BASE:AAA)

| AD8232 | M5Stack GPIO |
|--------|--------------|
| OUTPUT | GPIO 34 (ADC1_CH6) |
| LO+    | GPIO 13 |
| LO-    | GPIO 22 |
| 3.3V   | 3.3V |
| GND    | GND |
| SDN    | 未接続（常時動作） |

> ⚠️ **AD8232 の電源は 3.3V のみ。5V には絶対に接続しない。** モジュールが壊れる。
>
> ⚠️ **LO- に GPIO12 を使ってはいけない。** GPIO12 はESP32起動時のフラッシュ電圧 strap pin。AD8232 を繋いだまま電源を入れると high に引っ張られ、ESP32 が「フラッシュ電圧 1.8V」と誤判定してフラッシュと通信できなくなる（書き込みが `Failed to communicate with the flash chip` で必ず失敗、起動も不安定）。本プロジェクトは LO- を **GPIO22** に逃がしている。

## 電極装着 (Einthoven Lead I)

肌の脂を**アルコール綿でしっかり拭いて**から電極を貼る。これだけで信号品質が大きく変わる。

| 電極 | 位置 | リード線色（一般的） |
|------|------|----------------------|
| RA   | 右鎖骨の下、肩寄り | 赤 |
| LA   | 左鎖骨の下、肩寄り | 黄 |
| RL   | 右の下腹部 or 右腰骨付近（リファレンス） | 緑 or 黒 |

胸毛が濃い場合は剃ってから貼ったほうがいい。電極が浮くとノイズの塊になる。

## ビルド & 書き込み

PlatformIO Core を使う。公式インストーラスクリプトで `~/.platformio/penv` に専用 venv が入る。

### 初回セットアップ

```powershell
# PlatformIO Core を ~/.platformio/penv に入れる
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
python get-platformio.py
```

CP210x シリアルドライバが当たっていない場合は Silicon Labs のサイトから入れる（Windows 11 は通常自動）。
<https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers>

### ビルド・書き込み・モニタ

```powershell
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"

& $pio run -t upload          # ビルドして書き込み
& $pio device monitor          # シリアルモニタ (115200bps)
```

`platformio.ini` で `upload_port = COM3` / `upload_speed = 921600` 固定。別ポートに繋がる場合は書き換える。

書き込みは `LO-` が GPIO22 に逃げているので、**AD8232 を刺したままでもOK**。

## 使い方

1. 電極を貼り、AD8232 を接続。USB から外し、M5Stack のバッテリーで起動する。
2. **椅子に座って背筋を伸ばし、口を閉じて自然呼吸**。**そのまま 5分間動かない**。
3. 画面下段が `calibrating M:SS  n=X` のカウントダウンになる。
   - 5秒毎に 1サンプル蓄積（最大 60 サンプル＝5分間）
   - 5分経過 + 50 サンプル以上集まった時点でベースラインが自動採用される
4. ベースライン確定後は `base XX ms (rolling)` 表示に切り替わり、以後 5秒毎に **直近 5分間の中央値** でベースラインが滑らかに更新され続ける。LEADS OFF や体動で一時的にサンプルが欠けても、窓内の既存値で破綻しないようになっている。
5. 呼吸法／作業／緊張時などに、PNS プロキシバーの振れを観察。
   - 🟢 **緑** (`> 1.15x`): RMSSDがベースラインより上＝副交感寄り
   - 🟦 **シアン** (`0.85〜1.15x`): ほぼベースライン
   - 🟧 **オレンジ** (`< 0.85x`): RMSSDがベースラインより下＝交感寄り or ノイズ

### ボタン

| ボタン | 動作 |
|--------|------|
| **BtnA**（左） | キャリブレーション窓をクリアして最初からやり直し（5分待ち再開） |
| **BtnB**（中央）| `frozen` ⇄ `rolling` をトグル。`frozen` にした瞬間のベースラインで固定したいとき用 |
| **BtnC**（右） | LCD バックライトを **dim (≈0)** ⇄ **ON (128)** でトグル。起動直後は dim |

ログ ON/OFF はシリアルコマンド `LOG ON` / `LOG OFF` で切替（SD FAIL 時は ON 不可）。
画面右上の `LOG ON`（緑）／`LOG OFF`（灰）／`SD FAIL`（赤）は LCD が点灯しているときだけ見える。

電極が外れると `LEADS OFF` と表示され、波形描画もキャリブレーションのサンプル蓄積も止まる。電極を貼り直せば自然に再開する。

## 結果の解釈と交絡因子

RMSSDは敏感な指標で、生理状態以外の要因で簡単に変動する。
**測定条件を揃えないと比較は意味を持たない。**

- **呼吸の影響が一番大きい。** 深い・遅い呼吸では RMSSD が大きくなる（呼吸性洞性不整脈）。「副交感優位」というより呼吸条件の違いを見ているだけ、ということがよくある。
- **体動でノイズが出る。** 動くと差分が爆発して RMSSD が見かけ上跳ねる（本実装は >300ms 差分を除外する保険を入れているが完全ではない）。
- **時刻・カフェイン・直前の運動・食事・睡眠・気温**などすべてが効く。比較するなら **同じ時間帯・同じ姿勢・同じ呼吸条件** で。
- **絶対値の RMSSD は個人差が大きい。** 「他人の値と比べてどうか」は意味が薄い。自分のベースラインとの相対変化を見る装置として割り切る。

## MicroSD ログ

長期解析用に **MicroSD への CSV ログ**を取る。LCD 表示はオシロ的な「いま」を、SD ログは「あと」を見るための主データ。BLE はまだ実装していない。

### ログファイル

セッションごとに 2 つの CSV を作る。

| ファイル | 内容 | 更新タイミング |
|----------|------|----------------|
| `/rr_NNNNNN.csv` | R 波検出ごとの 1 行（RR 間隔、瞬時 BPM、その時の RMSSD など） | R 波検出毎 |
| `/summary_NNNNNN.csv` | 画面表示値のスナップショット（HR / RMSSD / baseline / ratio など） | 500 ms ごと |

`NNNNNN` は 6 桁ゼロ詰めの連番。既存ファイルを上書きしないよう、空いている番号を自動探索する。

I/O 効率のため、**ファイルは開きっぱなしで 5 秒ごとに flush** している。`sampleECG()` や ISR 内で SD アクセスはしない（loop 側で消費）。

### RR ログ列

```
session_ms,unix_time_ms,iso_time,rr_ms,bpm,rmssd_ms,baseline_ms,ratio,rr_count,quality,leads_off
```

| 列 | 意味 |
|----|------|
| `session_ms` | 起動からの経過 ms（時刻同期なしでも必ず入る） |
| `unix_time_ms` | Unix ms。時刻同期前は `0` |
| `iso_time` | ISO8601 UTC `YYYY-MM-DDTHH:MM:SS.mmmZ`。同期前は空 |
| `rr_ms` | 今回検出した RR 間隔 (ms) |
| `bpm` | `60000 / rr_ms` |
| `rmssd_ms` | その時点の RMSSD |
| `baseline_ms` | 採用ベースライン RMSSD。未確定は `0` |
| `ratio` | `rmssd / baseline_ms`。未確定は `0` |
| `rr_count` | RMSSD 計算に使えた RR の本数 |
| `quality` | 簡易品質指標。現状 `1.0` 固定（将来用） |
| `leads_off` | 0/1（R 波検出時点は常に 0） |

### Summary ログ列

```
session_ms,unix_time_ms,iso_time,bpm,rmssd_ms,baseline_ms,ratio,rr_count,leads_off,noise_count
```

`noise_count` は現状 `0` 固定（将来の RR 棄却カウント用）。

### 時刻同期（任意）

絶対時刻が不要なら `session_ms` だけで PC 側で十分解析できる。**絶対時刻を入れたい場合**はシリアルから `TIME <unix秒>` を1行送る：

```powershell
# PowerShell から現在時刻 (UTC) を送る例
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"
$unix = [int][double]::Parse((Get-Date -UFormat %s -Date (Get-Date).ToUniversalTime()))
"TIME $unix" | & $pio device monitor --echo
# または別のシリアル端末から「TIME 1738362600」のように改行付きで送る
```

ファームから `TIME SET:1738362600` が返れば成功。以降の `unix_time_ms` / `iso_time` 列が埋まる。
時刻同期は揮発で、リセットすると再設定が必要。

### 連続稼働時間

BASE:AAA + LCD 常時表示で **おおむね 3〜6時間**（バッテリー条件・LCD 明るさ依存）。終夜記録には外部給電が必要。

## 信号処理メモ

- **サンプリング**: 250 Hz、`hw_timer` ISR で `adc1_get_raw` を直接呼び、リングバッファに溜めて loop 側で消費。SPI 描画で遅延しても取りこぼしゼロ（シリアルの `DROPS` で監視可能）
- **ADC ノイズ対策**: ISR から渡された raw 値に **median-3** をかけてからベース除去へ
- **AD8232 内蔵フィルタ**: 約 0.5〜40 Hz BPF + 60 Hz ノッチ（モジュール側で完結）
- **ベースライン除去**: 指数移動平均 (α=0.002) で DC ドリフトを追従
- **起動ロックアウト**: 起動から 2.5 秒は AD8232 内蔵 BPF 収束を待つため R 波検出を行わない（`settling...` 表示）
- **R波検出**: ベース除去後の絶対値に対する自己適応閾値ピーク検出（不応期 300 ms = HR 上限 200 bpm）
- **振幅自動スケール**: `ampEma` (α=0.005) で直近の信号強度を追跡し、波形表示を画面に収める。クリップ時は ECG 帯の上下端に黄色ドット
- **RMSSD**: 直近最大128 RR間隔。外れ値除去は **2段階**：固定上限 `|ΔRR| < 300 ms` ＋ **Malik 20%**（`|ΔRR| < 0.2 × RR_prev`）
- **自動キャリブレーション**: 5秒毎に 1サンプル、最大 60 サンプル（5分間）の **rolling 中央値** をベースラインに採用。起動から 5分経過 + サンプル数 ≥ 50 で確定し、以後 5秒毎に更新
- **PNSプロキシバー**: `RMSSD / RMSSD_baseline` を log2 スケールで [0.5x, 2.0x] に正規化

## シリアル仕様 (460800 bps)

ECG 生波形まで含めるためボーレートを **460800 bps** に上げている（旧 115200 から変更）。`platformio.ini` の `monitor_speed = 460800` を参照。

すべての行は **行頭1文字でレコード種別**を表す統一フォーマット：

| 種別 | フォーマット | 頻度 |
|------|--------------|------|
| `I` | `I,<session_ms>,<unix_ms>,<event>[,<param>]` | イベント時 |
| `S` | `S,<session_ms>,<unix_ms>,<bpm>,<rmssd>,<base>,<calibn>,<frozen>,<drops>,<leads_off>` | 500 ms 毎 |
| `R` | `R,<session_ms>,<unix_ms>,<rr_ms>,<bpm>,<rmssd>,<base>,<ratio>,<rr_count>,<leads_off>` | R 波検出毎 |
| `E` | `E,<session_ms>,<raw>` | 250 Hz（デフォルト ON、`ECG OFF` で停止） |

`<unix_ms>` は `TIME` 同期前は `0`。

主なイベントコード：`BOOT` / `SD_OK` / `SD_FAIL` / `RR_LOG` / `SUM_LOG` / `LOG_ON` / `LOG_OFF` / `LOG_FAIL` / `TIME_SET` / `ECG_ON` / `ECG_OFF` / `LCD,DIM` / `LCD,ON`

### PC → ファーム コマンド

行末は `\n` または `\r\n`。

| コマンド | 動作 |
|----------|------|
| `TIME <unix_sec>` | 絶対時刻同期。応答に `I,...,TIME_SET,<unix_sec>` |
| `ECG ON` / `ECG OFF` | ECG 生波形 (E行) ストリームの ON/OFF |
| `LOG ON` / `LOG OFF` | MicroSD ログの ON/OFF（旧 BtnC 機能、SD FAIL 時は `LOG_FAIL`） |

### 帯域目安

ECG 250Hz ON 時で約 5 KB/s ≈ 40 kbps、460800bps の 9% 程度。シリアル使用率はゆとりがあり、HW FIFO ブロックは発生しない。

## PC でリアルタイム取得（AI 分析用）

`tools/monitor.py` でシリアルを PC 側 CSV に切り出します。Claude Code / Codex に「`data/latest/summary.csv` を分析して」と依頼すれば、そのままデータを読んで解析させられます。

### 依存

```powershell
pip install pyserial
# or use the platformio venv that already has it:
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m pip show pyserial
```

### 実行

```powershell
$py = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"

# 標準: COM3, 460800bps, TIME 自動同期, ECG raw も保存
& $py tools/monitor.py

# ECG raw は要らない（ファイル肥大を避けたい）
& $py tools/monitor.py --no-ecg

# 別ポート / 別出力先
& $py tools/monitor.py --port COM4 --data-dir D:/hrv
```

Ctrl+C で停止。各 CSV は **行バッファで常時 flush** されるので、途中まででも安全に解析できる。

### 出力ファイル

```
data/
├── session_20260518_220147/
│   ├── ecg.csv       (250 Hz の wall_iso, session_ms, raw)
│   ├── rr.csv        (R波毎)
│   ├── summary.csv   (500 ms 毎、画面表示と同じ値)
│   └── events.csv    (BOOT/SD_OK/TIME_SET など)
└── latest -> session_20260518_220147/   (シンボリックリンク、Windowsで権限不足ならLATEST.txt)
```

`data/latest/summary.csv` のヘッダ：

```csv
wall_iso,session_ms,unix_ms,bpm,rmssd,base,calibn,frozen,drops,leads_off
2026-05-18T13:01:48.863+00:00,540,1779109308540,72.4,43.1,38.0,60,0,0,0
...
```

### AI 分析のサンプル指示

```
data/latest/summary.csv の直近10分の rmssd/base 比を見て、
ベースラインから外れている区間と、その時の leads_off=0 を確認して、
ストレス傾向を要約して。
```

### 注意

- 起動直後の数百バイトは ESP32 ROM bootloader が **74880 bps** で出すメッセージなので、PC 側からは文字化けして `events.csv` に `RAW` 行として保存される（無害、捨ててOK）
- PC ロガーが落ちても **MicroSD 側のログは継続する**（フォールバック）。逆に SD が無くても PC 側 CSV は取れる
- `wall_iso` は PC 側で付ける UTC ISO8601。`unix_ms` はファーム側で `TIME` 同期後に入る。両方残しているのは同期前後の照合用

## LCD 明るさ

実行時は **BtnC** で `LCD_BRIGHTNESS_DIM` (0) ⇄ `LCD_BRIGHTNESS_ON` (128) を切替。起動時は `DIM` で始まる（省電力 / 暗所向け）。

シリアルにも `I,...,LCD,DIM` / `I,...,LCD,ON` のイベントが出る。

明るさ値を恒久的に変えたい場合は `src/main.cpp` の：

```cpp
constexpr uint8_t LCD_BRIGHTNESS_DIM = 0;     // 0 だと完全消灯に近い
constexpr uint8_t LCD_BRIGHTNESS_ON  = 128;   // 通常は 64..160 あたりが見やすい
```

を編集して再ビルド。

## ファイル構成

```
rmssd-ad8232/
├── platformio.ini              ; m5stack-core-esp32, upload=921600, monitor=460800
├── src/main.cpp                ; 250Hz ISR + R波検出 + RMSSD + LCD + SD + シリアルストリーム
├── tools/
│   ├── monitor.py              ; PC側ロガー (pyserial)
│   └── requirements.txt        ; pyserial>=3.5
├── data/                       ; (gitignore) PCロガー出力先
│   ├── session_YYYYMMDD_HHMMSS/
│   │   ├── ecg.csv, rr.csv, summary.csv, events.csv
│   └── latest -> session_YYYYMMDD_HHMMSS/
├── README.md
├── ECG HRV viewer wiring and guide.png
└── .gitignore
```

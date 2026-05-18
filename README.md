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
| M5Stack Basic | ESP32 + 320x240 LCD |
| AD8232 ECG モジュール | Keyestudio 版（本プロジェクトの配線表はこれ前提）。SparkFun 版はピン名が `SDN`/`3.3V` などで少し違うので適宜読み替え |
| 使い捨て ECG 電極 × 3 | スナップ式（バイオセンスやアンブー製のホルター用ゲル電極）。 |
| 電極リード線 | AD8232 付属のスナップ・モノラルジャック3本（赤=RA, 黄=LA, 緑/黒=RL） |
| ジャンパーワイヤー数本 | M5Stack BASE:AAA に挿す用 |

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
2. **椅子に座って背筋を伸ばし、口を閉じて自然呼吸**。1〜2分そのまま待つ。
3. `HR`／`RMSSD` が安定したら **BtnA**（左ボタン）を押してベースライン登録。
4. 呼吸法／作業／緊張時などに、PNS プロキシバーがどう振れるかを観察。
   - 🟢 **緑** (`> 1.15x`): RMSSDがベースラインより上＝副交感寄り
   - 🟦 **シアン** (`0.85〜1.15x`): ほぼベースライン
   - 🟧 **オレンジ** (`< 0.85x`): RMSSDがベースラインより下＝交感寄り or ノイズ
5. ベースラインをやり直したいときは **BtnB**（中央ボタン）でクリア。

電極が外れると `LEADS OFF` と表示され、波形描画が止まる。

## 結果の解釈と交絡因子

RMSSDは敏感な指標で、生理状態以外の要因で簡単に変動する。
**測定条件を揃えないと比較は意味を持たない。**

- **呼吸の影響が一番大きい。** 深い・遅い呼吸では RMSSD が大きくなる（呼吸性洞性不整脈）。「副交感優位」というより呼吸条件の違いを見ているだけ、ということがよくある。
- **体動でノイズが出る。** 動くと差分が爆発して RMSSD が見かけ上跳ねる（本実装は >300ms 差分を除外する保険を入れているが完全ではない）。
- **時刻・カフェイン・直前の運動・食事・睡眠・気温**などすべてが効く。比較するなら **同じ時間帯・同じ姿勢・同じ呼吸条件** で。
- **絶対値の RMSSD は個人差が大きい。** 「他人の値と比べてどうか」は意味が薄い。自分のベースラインとの相対変化を見る装置として割り切る。

## ファイル構成

```
rmssd-ad8232/
├── platformio.ini              ; m5stack-core-esp32, COM3, 921600bps
├── src/main.cpp                ; サンプリング(250Hz) + R波検出 + RMSSD + 描画
├── README.md
├── ECG HRV viewer wiring and guide.png
└── .gitignore
```

## 信号処理メモ

- **サンプリング**: 250 Hz (ADC1_CH6 = GPIO34, 12bit, 11dB)
- **AD8232 内蔵フィルタ**: 約 0.5〜40 Hz BPF + 60 Hz ノッチ（モジュール側で完結）
- **ベースライン除去**: 指数移動平均 (α=0.002) で DC ドリフトを追従
- **R波検出**: ベース除去後の絶対値に対する自己適応閾値ピーク検出（最低 RR = 250 ms）
- **振幅自動スケール**: `ampEma` (α=0.005) で直近の信号強度を追跡し、波形表示を画面に収める
- **RMSSD**: 直近最大128 RR間隔、差分絶対値 > 300 ms はノイズとして除外して二乗平均平方根
- **PNSプロキシバー**: `RMSSD / RMSSD_baseline` を log2 スケールで [0.5x, 2.0x] に正規化

## シリアル出力

500 ms 毎に CSV ライクで出力される。SD 保存も Wi-Fi 送信もしないので、長時間ロギングしたい場合は PC 側で `pio device monitor` の出力を `Tee-Object` 等で受ける。

```
BPM:72.4,RMSSD:43.1,BASE:38.0
BPM:71.9,RMSSD:43.5,BASE:38.0
...
```

```powershell
& $pio device monitor | Tee-Object -FilePath hrv_log.csv
```

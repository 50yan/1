# 天安门 — Design Spec

> 天安门是中国最具标志性的国家象征，始建于明朝永乐年间，六百年来承载着中华民族的尊严与荣耀。

## I. Project Information

| Item | Value |
| ---- | ----- |
| **Project Name** | 天安门 |
| **Canvas Format** | PPT 16:9 (1280×720) |
| **Page Count** | 1 |
| **Design Style** | General Versatile + 文化纪实 |
| **Target Audience** | 大众观众、文化爱好者、来访游客 |
| **Use Case** | 文化介绍、旅游宣传、历史教育 |
| **Created Date** | 2026-06-10 |

---

## II. Canvas Specification

| Property | Value |
| -------- | ----- |
| **Format** | PPT 16:9 |
| **Dimensions** | 1280×720 |
| **viewBox** | `0 0 1280 720` |
| **Margins** | left/right 50px, top/bottom 40px |
| **Content Area** | 1180×640 |

---

## III. Visual Theme

### Theme Style

- **Style**: General Versatile + 文化纪实
- **Theme**: Light theme
- **Tone**: 庄重、大气、纪实、典雅

### Color Scheme

| Role | HEX | Purpose |
| ---- | --- | ------- |
| **Background** | `#F5F0E8` | 米白宣纸底色，温润典雅 |
| **Secondary bg** | `rgba(245,240,232,0.85)` | 半透明卡片区域背景 |
| **Primary** | `#C41E3A` | 中国红，天安门标志色，标题装饰 |
| **Accent** | `#D4A017` | 金色，呼应城楼金瓦琉璃 |
| **Secondary accent** | `#8B0000` | 深红，层次过渡 |
| **Body text** | `#1A1A1A` | 正文深黑 |
| **Secondary text** | `#666666` | 次要信息灰色 |
| **Border/divider** | `#D4A017` | 金色分割线 |

### AI Image Strategy

- **Image Rendering**: editorial
- **Image Palette**: editorial-classic
- **Image Rendering Behavior**: Magazine-level editorial photography style, realistic lighting and natural shadows, immersive scene presence
- **Image Palette Behavior**: Off-white `#F5F0E8` paper base ~60% as breathing field, Chinese red `#C41E3A` main subject ~30% anchoring the focal area, gold `#D4A017` accent ~10% as warm highlights; reads as a National Geographic Beijing feature spread

### Gradient Scheme

```xml
<!-- Scrim gradient for text legibility -->
<linearGradient id="scrimGradient" x1="0%" y1="0%" x2="0%" y2="100%">
  <stop offset="0%" stop-color="#000000" stop-opacity="0"/>
  <stop offset="70%" stop-color="#000000" stop-opacity="0.45"/>
  <stop offset="100%" stop-color="#000000" stop-opacity="0.6"/>
</linearGradient>

<!-- 金色装饰光晕 -->
<radialGradient id="goldGlow" cx="50%" cy="30%" r="60%">
  <stop offset="0%" stop-color="#D4A017" stop-opacity="0.25"/>
  <stop offset="100%" stop-color="#D4A017" stop-opacity="0"/>
</radialGradient>
```

---

## IV. Typography System

### Font Plan

- **Typography direction**: 文化纪实 — 宋体/楷体标题 × 黑体正文，典雅与实用的结合

| Role | Chinese | English | Fallback tail |
| ---- | ------- | ------- | ------------- |
| **Title** | `KaiTi`, `SimSun` | `Georgia` | `serif` |
| **Body** | `"Microsoft YaHei"`, `"PingFang SC"` | `Arial` | `sans-serif` |
| **Emphasis** | `KaiTi`, `SimSun` | `Georgia` | `serif` |
| **Code** | — | `Consolas`, `"Courier New"` | `monospace` |

**Per-role font stacks** (CSS `font-family` strings):

- Title: `Georgia, KaiTi, SimSun, serif`
- Body: `"Microsoft YaHei", "PingFang SC", Arial, sans-serif`
- Emphasis: `Georgia, KaiTi, Serif`
- Code: `Consolas, "Courier New", monospace`

### Font Size Hierarchy

**Baseline**: Body font size = 18px (内容密度适中)

| Purpose | Ratio to body | Size | Weight |
| ------- | ------------- | ---- | ------ |
| Cover title (hero headline) | 3.3x | 60px | Bold |
| Subtitle | 1.5x | 27px | SemiBold |
| **Body content** | **1x** | **18px** | Regular |
| Page footer / decoration | 0.7x | 13px | Regular |

---

## V. Layout Principles

### Page Structure

- **Image area**: Full-bleed background 1280×720, with bottom scrim gradient
- **Title area**: text overlay bottom-center, above the scrim
- **Content area**: information cards or text blocks overlaid on the lower portion

### Layout Pattern

| Pattern | Usage |
| ------- | ----- |
| **#1 Full-bleed background with floating title** | Background image fills canvas; title floats over scrim |
| **#12 Faded image + text overlay** | Scrim ensures text legibility |
| **#29 Two-stop scrim** (Modifier) | Gradient darkens bottom 40% for text readability |

### Spacing Specification

| Element | Value |
| ------- | ----- |
| Safe margin from canvas edge | 50px |
| Content block gap | 30px |
| Card padding | 25px |
| Card border radius | 12px |

---

## VI. Icon Usage Specification

### Source

- **Built-in icon library**: `chunk-filled`
- **Usage method**: SVG placeholder `<use data-icon="chunk-filled/icon-name" .../>`

### Recommended Icon List

| Purpose | Icon Path | Page |
| ------- | --------- | ---- |
| 天安门城楼标志 | `chunk-filled/landmark` | Slide 01 |
| 地理位置 | `chunk-filled/map-pin` | Slide 01 |

---

## VII. Visualization Reference List

Not applicable — single-page deck with no data visualization.

---

## VIII. Image Resource List

| Filename | Dimensions | Ratio | Purpose | Type | Layout pattern | Acquire Via | Status | Reference | text_policy |
| -------- | ---------- | ----- | ------- | ---- | -------------- | ----------- | ------ | --------- | ----------- |
| bg_tiananmen.png | 1280×720 | 1.78 | 天安门城楼与广场的全景背景 — 庄严肃穆 | Photography | #1 full-bleed background with floating title + #29 two-stop scrim | ai | Pending | 天安门城楼正面全景，红色城墙金色琉璃瓦，蓝天白云，广场开阔，画面庄重肃穆，中央留有足够空间供文字标题叠加 | none |

---

## IX. Content Outline

### Slide 01 — 天安门

- **Layout**: #1 Full-bleed background + floating title over scrim + information cards
- **Title**: 天安门
- **Subtitle**: 中华人民共和国象征 | 世界文化遗产
- **Content**:
  - **天安门广场**: 世界最大的城市广场，面积44万平方米，可容纳百万人集会。人民英雄纪念碑、毛主席纪念堂、国家博物馆、人民大会堂环绕四周。
  - **天安门城楼**: 始建于明永乐十五年（1417年），原名"承天门"，清顺治八年（1651年）重建改称"天安门"。城楼高34.7米，面阔九间，进深五间，象征"九五至尊"。
  - **历史意义**: 1949年10月1日，毛泽东主席在此宣告中华人民共和国成立。天安门成为了新中国诞生的见证者，承载着中华民族的复兴梦想与爱国情怀。

---

## X. Speaker Notes Requirements

- **Filename**: `01_tiananmen.md`
- **Content**: 以庄重沉稳的语调介绍天安门的历史与象征意义。过渡从容，每段信息间隔2-3秒。整体时长约2分钟。

---

## XI. Technical Constraints Reminder

### SVG Generation Must Follow:

1. viewBox: `0 0 1280 720`
2. Background uses `<rect>` elements
3. Text wrapping uses `<tspan>` (`<foreignObject>` FORBIDDEN)
4. Transparency uses `fill-opacity` / `stroke-opacity`; `rgba()` FORBIDDEN
5. FORBIDDEN: `mask`, `<style>`, `class`, `foreignObject`
6. FORBIDDEN: `textPath`, `animate*`, `script`
7. Text characters: write typography & symbols as raw Unicode; HTML named entities FORBIDDEN
8. `clipPath` conditionally allowed only on `<image>` elements

### PPT Compatibility Rules:

- `<g opacity="...">` FORBIDDEN
- Image transparency uses overlay mask layer
- Inline styles only; external CSS FORBIDDEN

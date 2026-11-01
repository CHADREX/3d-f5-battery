# 3d-f5-battery

Параметрические 3D-модели аккумуляторов iPhone (линейка F5ENERGY) под рендер.
Геометрия собирается скриптом в Blender (bpy) из JSON-спецификации в миллиметрах,
на выходе FBX / GLB / .blend + UV-шаблоны для текстурирования.

## Структура

```
specs/            спецификации моделей (мм): контур, толщина, шлейфы, коннекторы
scripts/          build_battery.py — генератор (bpy), экспорт, превью-рендеры
models/<id>/      результат: .fbx .glb .blend, uv_template_*.png, preview_*.png
reference/        dimensions.md — сводка размеров по всей линейке; previous_renders — старые рисованные версии
assets/           сюда кладутся текстуры (не входят в генерацию)
```

## Запуск

```bash
pip install bpy==5.2.3 pillow        # Blender как python-модуль (Python 3.11–3.13)
python3 scripts/build_battery.py specs/iphone_11_pro_max.json --out models
python3 scripts/build_battery.py specs/iphone_11_pro_max.json --out models --render --samples 64
```

Можно и из обычного Blender: `blender -b -P scripts/build_battery.py -- specs/iphone_11_pro_max.json`.

Проверка экспорта (обратный импорт FBX/GLB, размеры, материалы, рендер глянцем на артефакты шейдинга):

```bash
python3 scripts/verify_fbx.py models/iphone_11_pro_max/iphone_11_pro_max.fbx --out /tmp/verify.png
```

Нормали `Cell` — custom, взвешенные по площади граней (аналог Weighted Normal), поэтому большая
плоская грань шейдится ровно, а фаска плавно уходит в бок. Не пересчитывайте нормали после импорта.

## Объекты и материалы

| Объект | Материалы | UV |
|---|---|---|
| `Cell` | `Cell_Front`, `Cell_Back`, `Cell_Side` | перед: u = x/ширина, v = y/высота по габариту контура; зад: то же, но зеркально по u (текстура рисуется «как видно сзади»); бок: u = обход периметра, v = толщина. Вся фаска относится к переду/заду, `Cell_Side` только на вертикальной полосе |
| `Seal_*` | `Cell_Seal` | планарная; плоские «губы» запайки пакета, из которых выходят шлейфы |
| `Side_Wrap` | `Cell_Seal` | планарная; тёмное скруглённое ребро («бортик») вдоль внешнего края широкой части |
| `Flex_Main`, `Flex_Lower` | `Flex` | планарная по габариту шлейфа; узкая лента (~1.3 мм) по траектории с U-петлёй у выхода из запайки |
| `Pad_*` | `Stiffener` | планарная; полиимидный язычок под коннектором |
| `Connector_*` | `Connector_Plastic`, `Connector_Metal`, `Connector_Gold` | box; со стороны этикетки видна металлическая пластина-экран, разъём с золочёными контактами смотрит на обратную сторону |

Текстура для `Cell_Front` / `Cell_Back` — это прямоугольник с пропорцией ширина:высота
габарита (для 11 Pro Max 64.6:109, шаблон 1214×2048 px лежит в `models/<id>/uv_template_*.png`).
Фон чёрный, контент белый; где L-вырез — просто чёрное.

Единицы: сцена в метрах с отображением в мм, FBX экспортируется в см (стандарт Blender, Y-up),
GLB — в метрах.

## Добавить модель

1. Скопировать `specs/iphone_11_pro_max.json`, поменять `id`, `outline`, `thickness`, шлейфы.
   Типы контура: `L` (узкая часть слева сверху), `L_mirror`, `rect`, `polygon` (произвольный, с `fillets`).
2. Шлейф задаётся осевой линией `path` (точки в мм) с радиусами скруглений `radii` и шириной `width`;
   `pad` — язычок под коннектором (`center`, `size`, `radius`, `rotation_deg`); `connector.center/size/rotation_deg` — разъём.
   Старый формат `polygon` + `fillets` тоже поддерживается.
3. `connector_side`: `back` (контакты к плате, со стороны этикетки видна металлическая пластина, как на фото iFixit) или `front`.
   `seal_lips` — прямоугольники губ запайки, `side_wraps` — рёбра-бортики, `flex_z` — высота плоскости шлейфа (число в мм или `mid`).
4. Прогнать скрипт, посмотреть `preview_*.png`, поправить числа.

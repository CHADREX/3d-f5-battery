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

## Объекты и материалы

| Объект | Материалы | UV |
|---|---|---|
| `Cell` | `Cell_Front`, `Cell_Back`, `Cell_Side` | перед: u = x/ширина, v = y/высота по габариту контура; зад: то же, но зеркально по u (текстура рисуется «как видно сзади»); бок: u = обход периметра, v = толщина |
| `Flex_Main`, `Flex_Lower` | `Flex` | планарная по габариту шлейфа |
| `Stiffener_*` | `Stiffener` | box |
| `Connector_*` | `Connector_Plastic`, `Connector_Metal`, `Connector_Gold` | box |

Текстура для `Cell_Front` / `Cell_Back` — это прямоугольник с пропорцией ширина:высота
габарита (для 11 Pro Max 64.6:109, шаблон 1214×2048 px лежит в `models/<id>/uv_template_*.png`).
Фон чёрный, контент белый; где L-вырез — просто чёрное.

Единицы: сцена в метрах с отображением в мм, FBX экспортируется в см (стандарт Blender, Y-up),
GLB — в метрах.

## Добавить модель

1. Скопировать `specs/iphone_11_pro_max.json`, поменять `id`, `outline`, `thickness`, шлейфы.
   Типы контура: `L` (узкая часть слева сверху), `L_mirror`, `rect`, `polygon` (произвольный, с `fillets`).
2. Полигон шлейфа задаётся точками в мм в той же системе координат; `stiffener` — прямоугольник
   площадки под коннектором; `connector.center/size/rotation_deg` — коннектор.
3. `connector_side`: `back` (контакты от стороны с логотипом, как в телефоне) или `front`.
4. Прогнать скрипт, посмотреть `preview_*.png`, поправить числа.

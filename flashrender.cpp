#include "flashrender.h"
#include "flashlocker.h"
#include <QDir>
#include <QtConcurrent/QtConcurrent>
#include <numeric>
#include <QPainterPath>

using namespace flashmath;

FlashRender::Map::Map(const QString& path):
    FlashMap(path)
{
}

bool FlashRender::Map::intersects(const QPolygonF& polygon_m) const
{
  for (auto border_m: borders_m)
    if (border_m.intersects(polygon_m))
      return true;
  return false;
}

void FlashRender::Map::clear()
{
  FlashLocker big_locker(&main_lock, FlashLocker::Write);
  if (!big_locker.hasLocked())
    return;
  FlashLocker small_locker(&tile_lock, FlashLocker::Write);
  if (!small_locker.hasLocked())
    return;
  FlashMap::clear();
  for (int i = 0; i < FlashClass::max_layer_count; i++)
    render_data[i].clear();
  job_object_count = 0;
  job_start_list.clear();
}

void FlashRender::Map::loadMainVectorTile(bool load_objects)
{
  FlashMap::loadMainVectorTile(load_objects);
  if (load_objects)
  {
    QWriteLocker big_locker(&main_lock);
    addCollectionToIndex(main);
    main.status = VectorTile::Loaded;
  }
}

void FlashRender::Map::loadVectorTile(int tile_idx)
{
  FlashMap::loadVectorTile(tile_idx);
  QWriteLocker small_locker(&tile_lock);
  addCollectionToIndex(tiles[tile_idx]);
  tiles[tile_idx].status = VectorTile::Loaded;
}

void FlashRender::Map::updateJobAdresses()
{
  int total_object_count = 0;
  for (auto layer: render_data)
    total_object_count += layer.count();

  job_object_count = total_object_count / job_count;

  int curr_obj_count = 0;
  job_start_list.clear();
  job_start_list += {0, 0};
  for (int layer_idx = -1; auto layer: render_data)
  {
    layer_idx++;
    for (int object_idx = -1; auto obj: layer)
    {
      object_idx++;
      Q_UNUSED(obj)
      curr_obj_count++;
      if (curr_obj_count >= job_object_count)
      {
        job_start_list += {layer_idx, object_idx};
        curr_obj_count = 0;
      }
    }
  }
}

void FlashRender::Map::addCollectionToIndex(VectorTile& collection)
{
  for (auto& obj: collection)
  {
    auto cl           = classes[obj.class_idx];
    int  l            = cl.layer;
    int  custom_layer = obj.attributes.value("layer").toInt();
    if (custom_layer != 0)
      l = std::clamp(FlashClass::max_layer_count - 5 + custom_layer,
                     0, FlashClass::max_layer_count - 1);
    render_data[l].append(&obj);
  }
  updateJobAdresses();
}

void FlashRender::Map::setObject(const FreeObject& obj)
{
  auto         obj_addr = FlashMap::addObject(obj);
  FlashObject* map_obj  = nullptr;
  if (obj_addr.tile_idx == 0)
    map_obj = &main[obj_addr.obj_idx];
  else
  {
    VectorTile& tile = tiles[obj_addr.tile_idx - 1];
    map_obj          = &tile[obj_addr.obj_idx];
  }
  auto cl = getClass(obj.first.class_idx);
  render_data[cl.layer].append(map_obj);
  updateJobAdresses();
}

QPoint FlashRender::meters2pix(QPointF coor_m) const
{
  auto rectm = getDrawRectM();
  int  pix_x = (coor_m.x() - rectm.left()) / mip;
  int  pix_y = (coor_m.y() - rectm.top()) / mip;
  return {pix_x, pix_y};
}
QPointF FlashRender::pix2meters(QPointF pix) const
{
  auto   rectm = getDrawRectM();
  double xm    = pix.x() * mip + rectm.left();
  double ym    = pix.y() * mip + rectm.top();
  return {xm, ym};
}

FlashRender::FlashRender(Settings _s)
{
  s                 = _s;
  int big_tile_side = tile_side * s.big_tile_multiplier;
  pixmap            = QPixmap{big_tile_side, big_tile_side};
  connect(this, &QThread::finished, this, &FlashRender::onFinished);
}

FlashRender::~FlashRender()
{
  wait();
}

void FlashRender::addMapPath(QString path)
{
  insertMapPath(maps.count(), path, false);
}

void FlashRender::removeMapPath(QString path)
{
  wait();
  for (auto& map: maps)
  {
    if (map->path == path)
    {
      map->clear();
      break;
    }
  }
}

void FlashRender::insertMapPath(int idx, QString path, bool load_now)
{
  auto map = new Map(path);
  map->loadMainVectorTile(load_now);
  wait();
  if (idx < 0)
    idx = maps.count();
  maps.insert(idx, map);
}

void FlashRender::requestTile(TileSpec t)
{
  if (isRunning())
    return;
  tile_spec = t;
  render();
}

void FlashRender::setLocalization(QString v)
{
  localization = "name:" + v;
}
QByteArray FlashRender::getTile(TileSpec t) const
{
  auto tile_name = getTileName(t);
  for (auto& big_tile: big_tiles)
  {
    if (big_tile.name == tile_name)
      return big_tile.data;
  }
  return QByteArray();
}

QVector<const FlashRender::Map*> FlashRender::getMaps() const
{
  QVector<const FlashRender::Map*> ret;
  for (auto& map: maps)
    ret.append(map);
  return ret;
}

void FlashRender::wait()
{
  QThreadPool().globalInstance()->waitForDone();
  QThread::wait();
}

QVector<FlashObject> FlashRender::getLoadedObjects(QString map_path)
{
  auto map = getMapPtrByPath(map_path);
  return map->getLoadedObjects();
}

void FlashRender::setObject(QString                       map_path,
                            const FlashMap::ObjectAddress addr,
                            const FreeObject&             obj)
{
  auto map = getMapPtrByPath(map_path);
  if (map)
    map->setObject(addr, obj);
}

QRectF FlashRender::getDrawRectM() const
{
  QSizeF size_m      = {pixmap.width() * mip, pixmap.height() * mip};
  QRectF draw_rect_m = {top_left_m.x(), top_left_m.y(),
                        size_m.width(), size_m.height()};
  return draw_rect_m;
}

void FlashRender::checkUnload()
{
  auto draw_rect_m  = getDrawRectM();
  int  loaded_count = 0;
  for (int i = -1; auto& map: maps)
  {
    i++;
    if (i == 0)
      continue;
    if (map->getMainTileStatus() == FlashMap::VectorTile::Loaded)
    {
      if (!needToLoadMap(map, draw_rect_m))
        if (loaded_count > s.max_loaded_maps_count)
          map->clear();
      loaded_count++;
    }
  }
}

bool FlashRender::needToLoadMap(const Map*    map,
                                const QRectF& draw_rect_m)
{
  if (map->getMainMip() > 0 && mip > map->getMainMip())
    return false;
  bool frame_intersects = map->intersects(draw_rect_m);
  if (!frame_intersects)
    return false;

  auto top_left_m = pix2meters({0, 0});
  auto bottom_right_m =
      pix2meters({(double)pixmap.width(), (double)pixmap.height()});
  auto frame_m = QRectF{top_left_m, bottom_right_m}.normalized();
  frame_m.adjust(-10000, -10000, 10000, 10000);

  if (map->intersects(frame_m))
    return true;
  return false;
}

void FlashRender::adjustImages()
{
  for (auto& map: maps)
  {
    auto cl_count = map->getClassCount();
    for (int class_idx = 0; class_idx < cl_count; class_idx++)
    {
      auto cl    = &map->getClass(class_idx);
      auto image = cl->image;
      auto w     = round(cl->width_mm / s.pixel_size_mm);
      if (!image.isNull() && cl->image.width() != w)
        image = image.scaledToWidth(w, Qt::SmoothTransformation);
      FlashClass new_cl = *cl;
      new_cl.image      = image;
      map->setClass(class_idx, new_cl);
    }
  }
}

void FlashRender::checkLoad()
{
  auto                   draw_rect_m = getDrawRectM();
  QVector<QFuture<void>> job_list;
  for (int i = -1; auto& map: maps)
  {
    i++;
    if (i == 0)
      continue;
    if (map->path.isEmpty())
      continue;
    auto map_rect_m = map->getFrame().toMeters();

    if (!needToLoadMap(map, draw_rect_m))
      continue;

    if (map->getMainTileStatus() == FlashMap::VectorTile::Null)
      map->loadMainVectorTile(true);
    if (map->getMainTileStatus() == FlashMap::VectorTile::Loaded)
    {
      if (needToLoadMap(map, draw_rect_m))
      {
        int    tile_side_count = sqrt(map->getTileCount());
        QSizeF tile_size_m = {map_rect_m.width() / tile_side_count,
                              map_rect_m.height() / tile_side_count};
        for (int tile_idx = 0; tile_idx < map->getTileCount();
             tile_idx++)
        {
          int    tile_idx_y = tile_idx / tile_side_count;
          int    tile_idx_x = tile_idx - tile_idx_y * tile_side_count;
          double tile_left =
              map_rect_m.x() + tile_idx_x * tile_size_m.width();
          double tile_top =
              map_rect_m.y() + tile_idx_y * tile_size_m.height();
          QRectF tile_rect_m = {{tile_left, tile_top}, tile_size_m};
          if (map->getTileStatus(tile_idx) ==
                  FlashMap::VectorTile::Null &&
              draw_rect_m.intersects(tile_rect_m) &&
              mip < map->getTileMip())
            job_list.append(QtConcurrent::run(
                map, &Map::loadVectorTile, tile_idx));
        }
      }
    }
  }
  adjustImages();
  for (auto job: job_list)
    job.waitForFinished();
}

QString FlashRender::getObjectName(const FlashObject* obj)
{
  auto name = obj->attributes.value(localization);
  if (name.isEmpty())
    name = obj->attributes.value("name");
  auto house_number = obj->attributes.value("addr:housenumber");

  if (name.isEmpty())
    name = house_number;
  else
    name += "\n" + house_number;

  return name;
}

void FlashRender::paintPointName(QPainter* p, const QString& text,
                                 const QColor& text_color)
{
  QRect rect;
  int   w = s.max_name_width_mm / s.pixel_size_mm;
  rect.setSize({w, p->font().pixelSize()});

  auto s              = p->font().pixelSize() / 4;
  int  hor_name_shift = -text.count() * s;
  rect.moveLeft(hor_name_shift);
  rect.moveTop(s);

  p->setPen(Qt::white);
  auto shifts = {-2, 0, 2};
  int  flags  = Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap |
              Qt::TextDontClip;

  for (auto x: shifts)
    for (auto y: shifts)
    {
      p->save();
      p->translate(x, y);
      p->drawText(rect, flags, text);
      p->restore();
    }

  p->setPen(text_color);
  p->drawText(rect, flags, text);
}

void FlashRender::paintLineName(QPainter* p, const QString& text,
                                const QColor& color)
{
  p->setPen(Qt::white);
  auto shifts = {-2, 0, 2};
  for (auto x: shifts)
    for (auto y: shifts)
      p->drawText(x, y, text);

  p->setPen(color);
  p->drawText(0, 0, text);
}

void FlashRender::paintAreaName(QPainter*               p,
                                const PolygonNameEntry& dte)
{
  p->setPen(Qt::white);
  auto shifts = {-2, 0, 2};
  for (auto x: shifts)
    for (auto y: shifts)
    {
      p->save();
      p->translate(x, y);
      p->drawText(dte.rect,
                  dte.alignment | Qt::TextWordWrap | Qt::TextDontClip,
                  dte.text);
      p->restore();
    }

  p->setPen(dte.cl->text);
  p->drawText(dte.rect,
              dte.alignment | Qt::TextWordWrap | Qt::TextDontClip,
              dte.text);
}

void FlashRender::addPolygonNameEntry(
    QVector<PolygonNameEntry>& polygon_name_entries,
    PolygonNameEntry           new_entry)
{
  bool can_fit = true;
  for (auto dte: polygon_name_entries)
  {
    if (dte.actual_rect.intersects(new_entry.actual_rect))
    {
      can_fit = false;
      break;
    }
  }
  if (can_fit)
    polygon_name_entries.append(new_entry);
};

FlashRender::TileSpec FlashRender::getBigTileCoor(TileSpec t)
{
  TileSpec bt;
  bt.x = int(t.x / s.big_tile_multiplier) * s.big_tile_multiplier;
  bt.y = int(t.y / s.big_tile_multiplier) * s.big_tile_multiplier;
  bt.z = t.z;
  return bt;
}

QString FlashRender::getTileName(TileSpec t) const
{
  return "z=" + QString("%1").arg(t.z) +
         ",y=" + QString("%1").arg(t.y) +
         ",x=" + QString("%1").arg(t.x) + +".bmp";
}

QPoint FlashRender::deg2pix(FlashGeoCoor kp) const
{
  auto m = kp.toMeters();
  return {int((m.x() - top_left_m.x()) / mip),
          int((m.y() - top_left_m.y()) / mip)};
}

QSize FlashRender::getRectSizePix(FlashGeoRect rect)
{
  auto obj_top_left     = deg2pix(rect.top_left);
  auto obj_bottom_right = deg2pix(rect.bottom_right);
  return QSize{obj_bottom_right.x() - obj_top_left.x(),
               obj_bottom_right.y() - obj_top_left.y()};
}

void FlashRender::paintPointObject(QPainter* p, const Map* map,
                                   const FlashObject* obj,
                                   int                job_idx)
{
  auto cl = &map->getClass(obj->class_idx);

  auto    kpos = obj->polygons.first().first();
  QPoint  pos  = deg2pix(kpos);
  QString str;

  auto name = getObjectName(obj);
  if (name.isEmpty() && cl->image.isNull() && mip < 1.0)
    name = cl->id;

  str += name;

  int w         = name.count() * p->font().pixelSize();
  int width_pix = round(cl->width_mm / s.pixel_size_mm);
  if (name.isEmpty())
    w = width_pix;

  auto name_rect = QRect{pos.x(), pos.y(), w, width_pix};
  point_names[job_idx].append({name_rect, str, cl});
}

QPolygon FlashRender::poly2pix(const FlashGeoPolygon& polygon) const
{
  QPolygon pl;
  pl.resize(polygon.count());
  for (int i = 0; i < polygon.count(); i++)
    pl[i] = deg2pix(polygon.at(i));
  return pl;
}

void FlashRender::paintAreaObject(QPainter* p, const Map* map,
                                  const FlashObject* obj, int job_idx)
{
  auto cl = &map->getClass(obj->class_idx);

  if (cl->pen == Qt::black)
    p->setPen(Qt::NoPen);
  else
    p->setPen(cl->pen);

  if (cl->style == FlashClass::BDiag)
    p->setBrush(QBrush(cl->brush, Qt::BDiagPattern));
  else if (cl->style == FlashClass::FDiag)
    p->setBrush(QBrush(cl->brush, Qt::FDiagPattern));
  else if (cl->style == FlashClass::Horiz)
    p->setBrush(QBrush(cl->brush, Qt::HorPattern));
  else if (cl->style == FlashClass::Vert)
    p->setBrush(QBrush(cl->brush, Qt::VerPattern));
  else if (cl->style == FlashClass::Dots)
    p->setBrush(QBrush(cl->brush, Qt::Dense7Pattern));
  else
  {
    if (cl->brush == Qt::black)
      p->setBrush(Qt::NoBrush);
    else
      p->setBrush(cl->brush);
  }

  auto    font_pixel_size = p->font().pixelSize();
  QString name            = getObjectName(obj);

  bool atleast_one_outer_polygon_inside = false;

  int polygon_count = obj->polygons.count();

  bool need_to_check_multipolygon_subs =
      (polygon_count > 1) && (obj->inner_polygon_start_idx >= 0);

  auto obj_frame_pix = getRectSizePix(obj->frame);

  QPainterPath path;
  QPainterPath inner_path;

  for (int polygon_idx = -1; auto& polygon: obj->polygons)
  {
    polygon_idx++;

    if (need_to_check_multipolygon_subs &&
        polygon_idx >= obj->inner_polygon_start_idx)
    {
      if (!atleast_one_outer_polygon_inside)
        return;
    }

    auto pl    = poly2pix(polygon);
    auto frame = pl.boundingRect();
    if (frame.width() < s.mix_visible_polygon_size_pix &&
        frame.height() < s.mix_visible_polygon_size_pix)
      continue;

    int max_object_name_length_pix =
        s.max_name_width_mm / s.pixel_size_mm;
    auto intersect_check_frame = frame.adjusted(
        -max_object_name_length_pix, -max_object_name_length_pix,
        max_object_name_length_pix, max_object_name_length_pix);

    if (!intersect_check_frame.intersects(pixmap.rect()))
      continue;

    if (need_to_check_multipolygon_subs &&
        polygon_idx < obj->inner_polygon_start_idx)
      atleast_one_outer_polygon_inside = true;

    auto polygon_frame = pl.boundingRect();

    auto polygon_w      = polygon_frame.width();
    auto name_width_pix = font_pixel_size * name.count();

    if (polygon_idx < obj->inner_polygon_start_idx ||
        obj->inner_polygon_start_idx < 0)
      if ((!name.isEmpty() && name_width_pix < polygon_w / 2) ||
          !cl->image.isNull())
      {
        auto  c = polygon_frame.center();
        QRect actual_rect;
        actual_rect.setTopLeft(c);
        actual_rect.setSize({name_width_pix, font_pixel_size});
        actual_rect.translate(
            {-name_width_pix / 2, -font_pixel_size / 2});

        addPolygonNameEntry(
            polygon_names[job_idx],
            {name, cl, polygon_frame, actual_rect, Qt::AlignCenter});
      }

    if (polygon_count == 1 || obj->inner_polygon_start_idx < 0)
    {
      p->drawPolygon(pl);
      continue;
    }

    auto pixmap_size_bytes =
        obj_frame_pix.width() * obj_frame_pix.height() * sizeof(int);

    if (polygon_count > s.max_path_polygon_count ||
        pixmap_size_bytes > 100000000 || mip > s.max_path_mip)
    {
      if (polygon_idx >= obj->inner_polygon_start_idx)
        p->setBrush(s.background_color);

      p->drawPolygon(pl);
    }
    else
    {
      if (polygon_idx < obj->inner_polygon_start_idx)
        path.addPolygon(pl);
      else
        inner_path.addPolygon(pl);
    }
  }
  if (!path.isEmpty())
  {
    path                 = path.subtracted(inner_path);
    auto    top_left_pix = deg2pix(obj->frame.top_left);
    QPixmap path_pm(getRectSizePix(obj->frame));
    path_pm.fill(Qt::transparent);
    QPainter path_painter(&path_pm);
    path_painter.setPen(p->pen());
    path_painter.setBrush(p->brush());
    path_painter.translate(-top_left_pix);
    path_painter.drawPath(path);
    p->drawPixmap(top_left_pix, path_pm);
  }
}

int FlashRender::getLineWidthPix(const FlashClass* cl,
                                 bool              scale_width) const
{
  int line_width_pix = round(cl->width_mm / s.pixel_size_mm);
  if (scale_width && mip < s.start_scaling_line_width_mip)
  {
    double coef = std::max(s.start_scaling_line_width_mip / mip, 1.0);
    line_width_pix *= coef;
  }
  if (cl->style != FlashClass::Solid)
    line_width_pix =
        std::min(line_width_pix, s.max_width_for_non_solid_lines_pix);
  return line_width_pix;
}

void FlashRender::paintLineObject(QPainter* painter, const Map* map,
                                  const FlashObject* obj, int job_idx)
{
  auto cl = &map->getClass(obj->class_idx);

  Qt::PenStyle style = Qt::SolidLine;
  if (cl->style == FlashClass::Dash)
    style = Qt::DashLine;
  if (cl->style == FlashClass::DashDot)
    style = Qt::DashDotLine;
  if (cl->style == FlashClass::Dots)
    style = Qt::DotLine;

  int obj_name_width = 0;

  auto name = getObjectName(obj);

  if (!name.isEmpty())
    obj_name_width = painter->font().pixelSize() * name.count() * 0.3;

  int line_width_pix = getLineWidthPix(cl, true);

  auto c = cl->pen;
  if (c == Qt::black)
    painter->setPen(Qt::NoPen);
  else
  {
    if (obj->attributes.contains("maxheight"))
      c.setAlpha(150);
    painter->setPen(
        QPen(c, line_width_pix, style, Qt::RoundCap, Qt::BevelJoin));
  }
  painter->setBrush(Qt::NoBrush);

  for (int poly_idx = -1; auto& polygon: obj->polygons)
  {
    poly_idx++;
    LineNameEntry nh;
    QPoint        p0;
    double        a0 = 0;

    auto pl = poly2pix(polygon);

    if (!name.isEmpty() && poly_idx == 0 && cl->pen != Qt::black)
      for (int point_idx = -1; auto p: pl)
      {
        point_idx++;
        if (nh.point_count == 0)
        {
          p0 = p;
          nh.point_count++;
          continue;
        }
        auto a = getAngle(p0, p);

        if (nh.point_count == 1)
          a0 = a;
        auto da = a0 - a;
        if (da > M_PI)
          da -= 2 * M_PI;
        else if (da < -M_PI)
          da += 2 * M_PI;
        da = fabs(da);
        if (da > deg2rad(5) || point_idx == polygon.count() - 1)
        {
          if (nh.length_pix > obj_name_width)
          {
            nh.fix(obj, pl.at(nh.start_idx), pl.at(nh.end_idx),
                   cl->text);
            line_names[job_idx].append(nh);
          }
          nh           = LineNameEntry();
          nh.start_idx = point_idx;
          p0           = p;
          continue;
        }
        auto length_pix = getDistance(p0, p);
        nh.length_pix += length_pix;
        p0 = p;
        nh.point_count++;
        nh.end_idx = point_idx;
      }
    painter->drawPolyline(pl);
  }
}

void FlashRender::LineNameEntry::fix(const FlashObject* _obj,
                                     const QPoint&      start,
                                     const QPoint&      end,
                                     const QColor&      _color)
{
  obj       = _obj;
  mid_point = {(start.x() + end.x()) / 2, (start.y() + end.y()) / 2};
  angle_deg = rad2deg(getAngle(start, end));
  if (angle_deg > 90)
    angle_deg -= 180;
  if (angle_deg < -90)
    angle_deg += 180;
  color = _color;
}

void FlashRender::AntiCluttering::clear()
{
  rects.clear();
  polygons.clear();
}

bool FlashRender::AntiCluttering::intersects(const QRect& new_rect)
{
  for (auto rect: rects)
    if (rect.intersects(new_rect))
      return true;
  for (auto polygon: polygons)
    if (polygon.intersects(new_rect))
      return true;
  return false;
}

bool FlashRender::AntiCluttering::intersects(
    const QPolygon& new_polygon)
{
  for (auto& rect: rects)
    if (new_polygon.intersects(rect))
      return true;
  for (auto& polygon: polygons)
    if (new_polygon.intersects(polygon))
      return true;
  return false;
}

bool FlashRender::AntiCluttering::check(const QRect& rect)
{
  if (intersects(rect))
    return false;
  rects.append(rect);
  return true;
}
bool FlashRender::AntiCluttering::check(const QPolygon& polygon)
{
  if (intersects(polygon))
    return false;
  polygons.append(polygon);
  return true;
}

bool FlashRender::checkMipRange(const FlashMap*    map,
                                const FlashObject* obj) const
{
  auto cl = &map->getClass(obj->class_idx);
  return (cl->min_mip == 0 || mip >= cl->min_mip) &&
         (cl->max_mip == 0 || mip <= cl->max_mip);
}

void FlashRender::paintObject(QPainter* p, const Map* map,
                              const FlashObject* obj, int job_idx)
{
  auto cl = &map->getClass(obj->class_idx);
  if (cl->style == FlashClass::Custom)
  {
    paintCustomObject(p, map, obj, mip);
    return;
  }

  switch (cl->type)
  {
  case FlashClass::Point:
    paintPointObject(p, map, obj, job_idx);
    break;
  case FlashClass::Line:
    paintLineObject(p, map, obj, job_idx);
    break;
  case FlashClass::Area:
    paintAreaObject(p, map, obj, job_idx);
    break;
  default:
    break;
  }
}

void FlashRender::paintPointNames(QPainter* p)
{
  for (int job_idx = 0; job_idx < job_count; job_idx++)
    for (auto item: point_names[job_idx])
    {
      if (!item.cl)
        continue;

      if (!anti_cluttering.check(item.rect))
        continue;

      auto pos           = item.rect.topLeft();
      auto font_size_pix = round(item.cl->width_mm / s.pixel_size_mm);

      if (font_size_pix > 0)
      {
        p->save();
        p->translate(pos);
        auto f = p->font();
        f.setPixelSize(font_size_pix);
        p->setFont(f);
        paintPointName(p, item.str, item.cl->text);
        p->restore();
      }
      if (item.cl->image.isNull())
      {
        if (item.cl->pen == Qt::black)
          p->setPen(Qt::NoPen);
        else
          p->setPen(item.cl->pen);

        if (item.cl->brush == Qt::black)
          p->setBrush(Qt::NoBrush);
        else
          p->setBrush(item.cl->brush);

        p->drawEllipse(pos, int(font_size_pix * 0.25),
                       int(font_size_pix * 0.25));
      }
      else
      {
        auto pos2 = QPoint{pos.x() - item.cl->image.width() / 2,
                           pos.y() - item.cl->image.height() / 2};
        p->drawImage(pos2, item.cl->image);
      }
    }
}

void FlashRender::paintLineNames(QPainter* p)
{
  QRect pixmap_rect = {0, 0, pixmap.width(), pixmap.height()};
  auto  f           = p->font();
  auto  w           = round(1.3 / s.pixel_size_mm);
  f.setPixelSize(w);
  p->setFont(f);

  for (int job_idx = 0; job_idx < job_count; job_idx++)
    for (auto nh: line_names[job_idx])
    {
      p->save();
      QRect   text_rect;
      QString name = getObjectName(nh.obj);
      int     text_width =
          int(p->font().pixelSize() * name.count() * 0.6);
      int text_height = int(p->font().pixelSize());
      text_rect.setSize({text_width, text_height});

      QTransform tr;
      tr.translate(nh.mid_point.x(), nh.mid_point.y());
      tr.rotate(nh.angle_deg);
      tr.translate(-text_rect.width() / 2, 0);

      auto mapped_rect = tr.mapRect(text_rect);
      if (!pixmap_rect
               .adjusted(-text_width * 2, -text_width * 2,
                         text_width * 2, text_width * 2)
               .contains(mapped_rect))
      {
        p->restore();
        continue;
      }
      auto mapped_polygon = tr.mapToPolygon(text_rect);
      if (!anti_cluttering.check(mapped_polygon))
      {
        p->restore();
        continue;
      }

      p->setTransform(tr);
      paintLineName(p, name, nh.color);
      p->restore();
    }
}

void FlashRender::paintAreaNames(QPainter* p)
{
  for (int job_idx = 0; job_idx < job_count; job_idx++)
    for (auto& entry: polygon_names[job_idx])
    {
      if (!anti_cluttering.check(entry.actual_rect))
        continue;

      QFontMetrics fm(p->font());
      paintAreaName(p, entry);

      if (!entry.cl->image.isNull() &&
          entry.rect.width() > entry.cl->image.width() * 2 &&
          entry.rect.height() > entry.cl->image.height() * 2)
      {
        auto obj_center = entry.rect.center();
        auto pos =
            QPoint{obj_center.x() - entry.cl->image.width() / 2,
                   obj_center.y() - entry.cl->image.height() / 2};

        auto image_width_pix =
            round(entry.cl->width_mm / s.pixel_size_mm);

        if (!entry.text.isEmpty())
          pos -= QPoint(0, image_width_pix + 5);

        p->drawImage(pos, entry.cl->image);
      }
    }
}

void FlashRender::renderJob(QPainter* p, QVector<Map*> render_maps,
                            int job_idx)
{
  for (auto map: render_maps)
  {
    FlashLocker main_locker(&map->main_lock, FlashLocker::Read);
    if (!main_locker.hasLocked())
      continue;
    FlashLocker tile_locker(&map->tile_lock, FlashLocker::Read);
    if (!tile_locker.hasLocked())
      continue;

    renderMap(p, map, job_idx);
  }
}

void FlashRender::renderMap(QPainter* p, const Map* map, int job_idx)
{
  if (!map || job_idx > map->job_start_list.count() - 1)
    return;

  auto            start = map->job_start_list[job_idx];
  Map::JobAddress end;
  if (job_idx < map->job_start_list.count() - 1)
    end = map->job_start_list[job_idx + 1];

  p->setRenderHint(QPainter::Antialiasing);

  int max_object_name_length_pix =
      s.max_name_width_mm / s.pixel_size_mm;
  auto clip_safe_rect_m =
      render_frame_m.adjusted(-max_object_name_length_pix * mip,
                              -max_object_name_length_pix * mip,
                              max_object_name_length_pix * mip,
                              max_object_name_length_pix * mip);

  FlashGeoCoor top_left_deg =
      FlashGeoCoor::fromMeters(clip_safe_rect_m.topLeft());
  FlashGeoCoor bottom_right_deg =
      FlashGeoCoor::fromMeters(clip_safe_rect_m.bottomRight());

  int  pixmap_frame_top_deg    = top_left_deg.lat;
  int  pixmap_frame_left_deg   = top_left_deg.lon;
  int  pixmap_frame_bottom_deg = bottom_right_deg.lat;
  int  pixmap_frame_right_deg  = bottom_right_deg.lon;
  auto frame_deg =
      QRect{QPoint{pixmap_frame_left_deg, pixmap_frame_top_deg},
            QPoint{pixmap_frame_right_deg, pixmap_frame_bottom_deg}};

  for (int layer_idx = start.layer_idx;
       layer_idx < FlashClass::max_layer_count; layer_idx++)
  {
    int start_obj_idx = 0;
    if (layer_idx == start.layer_idx)
      start_obj_idx = start.obj_idx;
    auto& layer     = map->render_data[layer_idx];
    auto  obj_count = layer.count();

    for (int obj_idx = start_obj_idx; obj_idx < obj_count; obj_idx++)
    {
      if (layer_idx == end.layer_idx && obj_idx == end.obj_idx + 1)
        return;

      auto& obj = layer[obj_idx];

      if (!obj)
        continue;

      auto cl    = &map->getClass(obj->class_idx);
      auto frame = &obj->frame;
      if (cl->type == FlashClass::Point)
      {
        if (!frame_deg.contains(
                {frame->top_left.lon, frame->top_left.lat}))
          continue;
      }
      else
      {
        auto obj_frame_deg = QRect{
            QPoint{frame->top_left.lon, frame->top_left.lat},
            QPoint{frame->bottom_right.lon, frame->bottom_right.lat}};

        qint64 x1 = obj_frame_deg.left();
        qint64 x2 = obj_frame_deg.right();
        qint64 w  = x2 - x1;

        if (w < INT_MAX / 2 && !frame_deg.intersects(obj_frame_deg) &&
            mip < 10000)
          continue;
      }

      if (!checkMipRange(map, obj))
        continue;

      paintObject(p, map, obj, job_idx);
    }
  }
}

struct JobEntry
{
  int            idx;
  QPixmap*       pm;
  QPainter*      p;
  QElapsedTimer* t;
  QFuture<void>* fut;
  JobEntry(int idx, QSize s, QFont* f);
  ~JobEntry();
};

JobEntry::JobEntry(int _idx, QSize size, QFont* f)
{
  idx = _idx;
  pm  = new QPixmap(size);
  pm->fill(Qt::transparent);
  p = new QPainter(pm);
  p->setFont(*f);
  t = new QElapsedTimer;
  t->start();
  fut = new QFuture<void>;
}

JobEntry::~JobEntry()
{
  delete p;
  delete pm;
  delete t;
  delete fut;
}

void FlashRender::onFinished()
{
  wait();
  big_tiles.append(big_tile);
  while (big_tiles.count() > 4 * sqr(s.big_tile_multiplier))
    big_tiles.removeFirst();
}

void FlashRender::run()
{
  QElapsedTimer total_render_time;
  total_render_time.start();
  int tile_count      = pow(2, tile_spec.z);
  int world_width_pix = tile_side * tile_count;
  mip = 2 * M_PI * flashmath::earth_r / world_width_pix;
  auto big_tile_coor = getBigTileCoor(tile_spec);
  top_left_m = {(big_tile_coor.x - tile_count / 2) * tile_side * mip,
                (big_tile_coor.y - tile_count / 2) * tile_side * mip};

  for (int i = 0; i < job_count; i++)
  {
    point_names[i].clear();
    polygon_names[i].clear();
    line_names[i].clear();
  }
  QSizeF size_m  = {pixmap.width() * mip, pixmap.height() * mip};
  render_frame_m = {top_left_m, size_m};

  checkLoad();

  QVector<int> intersecting_maps;
  auto         draw_rect = getDrawRectM();
  for (int map_idx = -1; auto& map: maps)
  {
    if (map->getMainMip() > 0 && mip > map->getMainMip())
      continue;
    map_idx++;
    if (map_idx == 0)
      continue;
    if (needToLoadMap(map, draw_rect))
      intersecting_maps.append(map_idx);
  }

  pixmap.fill(s.background_color);
  QPainter p0(&pixmap);

  QFont f = p0.font();

  double font_size =
      std::min((int)std::round(1.5 / s.pixel_size_mm / mip), 1);
  font_size = std::clamp(font_size, 1.5 / s.pixel_size_mm,
                         3.0 / s.pixel_size_mm);
  f.setPixelSize(font_size);
  f.setBold(true);
  p0.setFont(f);

  for (int iter = 0; iter < 2; iter++)
  {
    QVector<FlashRender::Map*> render_maps;
    for (int map_idx = -1; auto& map: maps)
    {
      map_idx++;

      if (iter == 1 && map_idx == 0)
        continue;

      if (map_idx > 0)
        if (!intersecting_maps.contains(map_idx))
          continue;

      FlashLocker big_locker(&map->main_lock, FlashLocker::Read);
      if (!big_locker.hasLocked())
        continue;

      if (map_idx > 0 && !needToLoadMap(map, render_frame_m))
        continue;

      if (map_idx > 0 && !map->intersects(render_frame_m))
        continue;

      FlashLocker small_locker(&map->tile_lock, FlashLocker::Read);
      if (!small_locker.hasLocked())
        continue;

      if (map->job_start_list.isEmpty())
        continue;

      render_maps.append(map);

      if (iter == 0)
        break;
    }

    QList<JobEntry*> render_list;

    for (int job_idx = 1; job_idx < job_count; job_idx++)
    {
      auto job  = new JobEntry(job_idx, pixmap.size(), &f);
      *job->fut = QtConcurrent::run(this, &FlashRender::renderJob,
                                    job->p, render_maps, job_idx);
      render_list.append(job);
    }

    renderJob(&p0, render_maps, 0);

    for (auto render: render_list)
      render->fut->waitForFinished();

    for (auto render: render_list)
      p0.drawPixmap(0, 0, *render->pm);

    qDeleteAll(render_list);
  }

  anti_cluttering.clear();
  paintLineNames(&p0);
  paintPointNames(&p0);
  paintAreaNames(&p0);

  big_tile.clear();
  for (int yi = 0; yi < s.big_tile_multiplier; yi++)
    for (int xi = 0; xi < s.big_tile_multiplier; xi++)
    {
      TileSpec t;
      t.x     = big_tile_coor.x + xi;
      t.y     = big_tile_coor.y + yi;
      t.z     = big_tile_coor.z;
      auto pm = pixmap.copy(xi * tile_side, yi * tile_side, tile_side,
                            tile_side);
      QByteArray ba;
      QBuffer    buf(&ba);
      pm.save(&buf, "bmp");

      big_tile.append({getTileName(t), ba});
    }
  qDebug() << "mip=" << mip;
  qDebug() << "render time=" << total_render_time.elapsed();
}

void FlashRender::render()
{
  if (isRunning())
    return;
  if (QThreadPool::globalInstance()->activeThreadCount() == 0)
    checkUnload();
  startedRender(getDrawRectM(), mip);
  QThread::start();
}

FlashMap* FlashRender::getMapPtrByPath(QString path)
{
  for (auto& map: maps)
    if (map->path == path)
      return map;
  return nullptr;
}

void FlashRender::clearTiles()
{
  big_tiles.clear();
}

FlashRender* FlashRender::getInst()
{
  auto ret = static_cast<FlashRender*>(
      qApp->findChild<QObject*>("FlashRender"));
  if (!ret)
    qDebug() << "Failed to get \"FlashRender\" object! Use "
                "setObjectName() for initialization";
  return ret;
}

#pragma once

#include "flashmap.h"
#include <QReadWriteLock>
#include <QThread>

class FlashRender: public QThread
{
  Q_OBJECT

public:
  struct TileSpec
  {
    int x, y, z;
  };

  static constexpr int job_count = 12;
  struct Map: public FlashMap
  {
    struct JobAddress
    {
      int layer_idx = -1;
      int obj_idx   = -1;
    };

    QVector<FlashObject*> render_data[FlashClass::max_layer_count];
    QReadWriteLock        main_lock;
    QReadWriteLock        tile_lock;
    QList<JobAddress>     job_start_list;
    int                   job_object_count;
    QString               path;

    void addCollectionToIndex(VectorTile& collection);
    void updateJobAdresses();

    Map(const QString& path);
    void clear();
    void loadMainVectorTile(bool load_objects);
    void loadVectorTile(int tile_idx);
    bool intersects(const QPolygonF& polygon) const;
    void setObject(const FreeObject&);
  };

  struct Settings
  {
    int    big_tile_multiplier               = 4;
    double pixel_size_mm                     = 0.1;
    double max_loaded_maps_count             = 3;
    double max_name_width_mm                 = 20.0;
    double start_scaling_line_width_mip      = 4.0;
    int    max_width_for_non_solid_lines_pix = 10;
    QColor background_color                  = {250, 246, 230};
    int    max_path_polygon_count            = 2000;
    int    max_path_mip                      = 50;
    int    mix_visible_polygon_size_pix      = 10;
  };

private:
  static constexpr int tile_side = 256;

  struct RenderResult
  {
    QString    name;
    QByteArray data;
  };

  struct PolygonNameEntry
  {
    QString           text;
    const FlashClass* cl;
    QRect             rect;
    QRect             actual_rect;
    Qt::Alignment     alignment;
  };

  struct LineNameEntry
  {

    int                length_pix  = 0;
    int                start_idx   = 0;
    int                end_idx     = 0;
    int                point_count = 0;
    double             angle_deg   = 0;
    QPoint             mid_point;
    const FlashObject* obj = nullptr;
    QColor             color;
    void fix(const FlashObject* obj, const QPoint& start,
             const QPoint& end, const QColor& color);
  };

  struct PointNameRect
  {
    QRect             rect;
    QString           str;
    const FlashClass* cl;
  };

  struct AntiCluttering
  {
    QVector<QRect>    rects;
    QVector<QPolygon> polygons;
    void              clear();
    bool              intersects(const QRect& rect);
    bool              intersects(const QPolygon& polygon);
    bool              check(const QRect& rect);
    bool              check(const QPolygon& polygon);
  };

  Settings s;
  TileSpec tile_spec;

  QPointF               top_left_m;
  double                mip;
  QPixmap               pixmap;
  QVector<RenderResult> big_tile;
  QVector<RenderResult> big_tiles;

  QVector<Map*> maps;

  QVector<PointNameRect>    point_names[job_count];
  QVector<PolygonNameEntry> polygon_names[job_count];
  QVector<LineNameEntry>    line_names[job_count];
  QRectF                    render_frame_m;
  QString                   localization;
  AntiCluttering            anti_cluttering;
  static void paintLineName(QPainter* p, const QString& text,
                            const QColor& color);

  void run();
  void start() = delete;
  void onFinished();

  void renderMap(QPainter* p, const Map* map, int job_idx);
  void renderJob(QPainter* p, QVector<Map*> render_maps, int job_idx);

  bool checkMipRange(const FlashMap*    map,
                     const FlashObject* obj) const;

  void paintObject(QPainter* p, const Map* map,
                   const FlashObject* obj, int job_idx);
  void paintPointNames(QPainter* p);
  void paintLineNames(QPainter* p);
  void paintAreaNames(QPainter* p);

  void paintAreaName(QPainter* p, const PolygonNameEntry& dte);
  void
  addPolygonNameEntry(QVector<PolygonNameEntry>& polygon_name_entries,
                      PolygonNameEntry           new_entry);

  QString   getObjectName(const FlashObject* obj);
  void      paintPointName(QPainter* p, const QString& text,
                           const QColor& text_color);
  void      paintPointObject(QPainter* p, const Map* map,
                             const FlashObject* obj, int job_idx);
  void      paintLineObject(QPainter* painter, const Map* map,
                            const FlashObject* obj, int job_idx);
  void      paintAreaObject(QPainter* p, const Map* map,
                            const FlashObject* obj, int job_idx);
  QRectF    getDrawRectM() const;
  bool      needToLoadMap(const Map* map, const QRectF& draw_rect);
  void      adjustImages();
  void      checkLoad();
  void      checkUnload();
  void      render();
  QPoint    meters2pix(QPointF m) const;
  QPointF   pix2meters(QPointF pix) const;
  QPoint    deg2pix(FlashGeoCoor) const;
  QSize     getRectSizePix(FlashGeoRect);
  TileSpec  getBigTileCoor(TileSpec);
  QString   getTileName(TileSpec) const;
  FlashMap* getMapPtrByPath(QString path);

signals:
  void startedRender(QRectF, double);
  void paintCustomObject(QPainter* painter, const Map* map,
                         const FlashObject* obj, double mip);

public:
  FlashRender(Settings);
  virtual ~FlashRender();
  void insertMapPath(int idx, QString path, bool load_now);
  void addMapPath(QString path);
  void removeMapPath(QString path);
  QVector<const FlashRender::Map*> getMaps() const;
  void                             wait();
  void setObject(QString map_path, const FlashMap::ObjectAddress addr,
                 const FreeObject&);
  QVector<FlashObject> getLoadedObjects(QString map_path);
  void                 requestTile(TileSpec);
  QByteArray           getTile(TileSpec) const;
  void                 setLocalization(QString);
  QPolygon             poly2pix(const FlashGeoPolygon& polygon) const;
  int  getLineWidthPix(const FlashClass* cl, bool scale_width) const;
  void clearTiles();
  static FlashRender* getInst();
};

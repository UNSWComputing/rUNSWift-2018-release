#include <QLabel>
#include <QPixmap>
#include <QResizeEvent>

/**
 * A QLabel for holding images
 *
 * Ensures they always disply in the correct aspect ratio
 */
class ImageView : public QLabel
{
    Q_OBJECT
    public:
        explicit ImageView(QWidget *parent = 0);
        virtual int heightForWidth(int width) const;
        virtual QSize sizeHint() const;
        QPixmap scaledPixmap() const;

    public slots:
        void setPixmap(const QPixmap &p);
        void resizeEvent(QResizeEvent *e);

    private:
        QPixmap pixmap;
        void applyPixmap();
};


#include "SimulateView.h"
#include "QtSimulateWindow.h"

#include <QVBoxLayout>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QUrl>
#include <QFileInfo>

SimulateView::SimulateView(QWidget *parent) : QWidget(parent) {
    m_window = new QtSimulateWindow();
    QWidget *container = QWidget::createWindowContainer(m_window, this);
    container->setMinimumSize(640, 480);
    container->setFocusPolicy(Qt::StrongFocus);

    auto *lay = new QVBoxLayout(this);
    lay->setContentsMargins(0,0,0,0);
    lay->addWidget(container);

    // 接受文件拖放：拖进 .xml / .mjb 即加载
    setAcceptDrops(true);
}

SimulateView::~SimulateView() = default;   // QtSimulateWindow 析构里会 stop()

void SimulateView::start(const QString &xmlPath) { m_window->start(xmlPath); }
void SimulateView::loadModel(const QString &xmlPath) { m_window->loadModel(xmlPath); }

static bool isModelFile(const QString &path) {
    QString suffix = QFileInfo(path).suffix().toLower();
    return suffix == "xml" || suffix == "mjb";
}

void SimulateView::dragEnterEvent(QDragEnterEvent *e) {
    if (!e->mimeData()->hasUrls()) return;
    for (const QUrl &u : e->mimeData()->urls()) {
        if (u.isLocalFile() && isModelFile(u.toLocalFile())) {
            e->acceptProposedAction();
            return;
        }
    }
}

void SimulateView::dropEvent(QDropEvent *e) {
    if (!e->mimeData()->hasUrls()) return;
    for (const QUrl &u : e->mimeData()->urls()) {
        if (u.isLocalFile() && isModelFile(u.toLocalFile())) {
            loadModel(u.toLocalFile());
            e->acceptProposedAction();
            return;
        }
    }
}

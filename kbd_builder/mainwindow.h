#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QButtonGroup>
#include <QPushButton>
#include <QGridLayout>
#include <QList>
#include <QMenu>

namespace Ui {
class MainWindow;
}

typedef struct
{
    int                 m_keys_row[256];
    int                 m_keys_col[256];
    int                 m_alts_row[256];
    int                 m_alts_col[256];
} kbd_mtx_t;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public slots:
    void btn_clicked(int id);
    void cell_triggered(bool fl);
    void cell_clear(bool fl);
private slots:
    void on_actionOpen_triggered();

    void on_actionSave_triggered();

    void on_actionSave_as_triggered();

    void on_actionLat_triggered();

    void on_actionCyr_triggered();

    void on_actionNew_triggered();

    void on_actionExport_header_triggered();

private:
    Ui::MainWindow      *ui;
    QButtonGroup        *m_buttons_container;
    QList<QPushButton*> m_buttons;
    QGridLayout         *m_lay;
    QMenu               *m_pop;
    QAction             *m_title;
    QList<QMenu*>       m_rows_menus;

    QString             m_file;

    QAbstractButton*    m_cur_btn;
    int                 m_cur_HID;
    int                 m_cur_mtx;
    kbd_mtx_t           m_kbd_mtx[2];

    QMap<QAction*,QPair<int,int>>   m_cells;
    QMap<int,QString>               m_strs;
    QMap<int,int>                   m_alts;

    void save_to_file();
    void update_ui();
};

#endif // MAINWINDOW_H

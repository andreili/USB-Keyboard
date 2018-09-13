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

private:
    Ui::MainWindow      *ui;
    QButtonGroup        *m_buttons_container;
    QList<QPushButton*> m_buttons;
    QGridLayout         *m_lay;
    QMenu               *m_pop;
    QAction             *m_title;
    QList<QMenu*>       m_rows_menus;

    QAbstractButton*    m_cur_btn;
    int                 m_cur_HID;
    int                 m_keys_row[256];
    int                 m_keys_col[256];
    int                 m_alts_row[256];
    int                 m_alts_col[256];

    QMap<QAction*,QPair<int,int>> m_cells;
};

#endif // MAINWINDOW_H

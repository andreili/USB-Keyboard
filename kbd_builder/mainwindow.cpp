#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>

#define KEY_ROWS 6
#define KEY_COLS 21
int keys_code[KEY_ROWS][KEY_COLS] =
    {{0x29,   -1, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,   -1,   -1,   -1,   -1},
     {0x35, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x2d, 0x2e, 0x2a, 0x49, 0x4a, 0x4b, 0x53, 0x54, 0x55, 0x56},
     {0x2b, 0x14, 0x1a, 0x08, 0x15, 0x17, 0x1c, 0x18, 0x0c, 0x12, 0x13, 0x2f, 0x30,   -1, 0x4c, 0x4d, 0x4e, 0x5f, 0x60, 0x61, 0x57},
     {0x39, 0x04, 0x16, 0x07, 0x09, 0x0a, 0x0b, 0x0d, 0x0e, 0x0f, 0x33, 0x34, 0x31, 0x28,   -1,   -1,   -1, 0x5c, 0x5d, 0x5e,   -1},
     {0x102,0x1d, 0x1b, 0x06, 0x19, 0x05, 0x11, 0x10, 0x36, 0x37, 0x38,   -1,   -1,0x120,   -1, 0x52,   -1, 0x59, 0x5a, 0x5b, 0x58},
     {0x101,0x108,0x104,  -1,   -1,   -1,   -1, 0x2c,   -1,   -1,   -1,0x140,0x180,0x110, 0x50, 0x51, 0x4f, 0x62,   -1, 0x63,   -1}};
QString keys_str[KEY_ROWS][KEY_COLS] =
    {{"Esc", "", "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12", "PrnScr", "ScrlLock", "Pause", "", "", "", ""},
     {"`~", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "=", "Bcksp", "Insert", "Home", "PgUp", "NumLock", "/", "*", "-"},
     {"Tab", "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "[", "]", "", "Del", "End", "PgDown", "7", "8", "9", "+"},
     {"CapsLock", "A", "S", "D", "F", "G", "H", "J", "K", "L", ";", "'", "\\", "Enter", "", "", "", "4", "5", "6", ""},
     {"LShift", "Z", "X", "C", "V", "B", "N", "M", ",", ".", "/", "", "", "RShift", "", "Up", "", "1", "2", "3", "Enter"},
     {"LCtrl", "LGUI", "LAlt", "", "", "", "", "Space", "", "", "", "RAlt", "RGUI", "RCtrl", "Left", "Down", "Right", "0", "", ".", ""}};

QString filter = "JSON text files (*.json)";

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_file = "";
    m_alts[0] = 0x101;
    m_alts[1] = 0x102;
    m_alts[2] = 0x104;
    m_alts[3] = 0x108;
    m_alts[4] = 0x110;
    m_alts[5] = 0x120;
    m_alts[6] = 0x140;
    m_alts[7] = 0x180;

    m_lay = new QGridLayout();
    m_buttons_container = new QButtonGroup();
    for (int i=0 ; i<KEY_ROWS ; ++i)
    {
        for (int j=0 ; j<KEY_COLS ; ++j)
        {
            int HID_code = keys_code[i][j];
            QPushButton *btn = new QPushButton(keys_str[i][j]);
            btn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
            m_lay->addWidget(btn, i, j);
            m_buttons.push_back(btn);
            if (HID_code != -1)
                m_buttons_container->addButton(btn, HID_code);
            m_strs[HID_code] = keys_str[i][j];
        }
    }

    connect(m_buttons_container, SIGNAL(buttonClicked(int)), this, SLOT(btn_clicked(int)));

    m_pop = new QMenu(this);
    m_title = m_pop->addAction("nothing");
    QAction* clr = m_pop->addAction("Clear");
    connect(clr, SIGNAL(triggered(bool)), this, SLOT(cell_clear(bool)));
    m_pop->addSeparator();

    for (int i=0 ; i<12 ; ++i)
    {
        QMenu* root = m_pop->addMenu("Row #" + QString::number(i));
        for (int j=0 ; j<12 ; ++j)
        {
            QAction* cell = root->addAction("Col #" + QString::number(j));
            connect(cell, SIGNAL(triggered(bool)), this, SLOT(cell_triggered(bool)));
            m_cells[cell] = QPair<int,int>(i, j);
        }
        m_rows_menus.push_back(root);
    }

    on_actionNew_triggered();
    ui->centralWidget->setLayout(m_lay);
    updateGeometry();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::btn_clicked(int id)
{
    m_cur_btn = m_buttons_container->button(id);
    m_cur_HID = id;

    m_title->setText("0x" + QString::number(id, 16));
    m_pop->popup(QCursor::pos());
}

void MainWindow::cell_triggered(bool fl)
{
    (void)(fl);
    QAction* cell = (QAction*)sender();
    QPair<int,int> cc = m_cells[cell];

    if (m_cur_HID < 0x100)
    {
        m_kbd_mtx[m_cur_mtx].m_keys_row[m_cur_HID] = cc.first;
        m_kbd_mtx[m_cur_mtx].m_keys_col[m_cur_HID] = cc.second;
    }
    else
    {
        m_kbd_mtx[0].m_alts_row[m_cur_HID - 0x100] = cc.first;
        m_kbd_mtx[0].m_alts_col[m_cur_HID - 0x100] = cc.second;
    }
    update_ui();
}

void MainWindow::cell_clear(bool fl)
{
    (void)(fl);
    if (m_cur_HID < 0x100)
    {
        m_kbd_mtx[m_cur_mtx].m_keys_row[m_cur_HID] = -1;
        m_kbd_mtx[m_cur_mtx].m_keys_col[m_cur_HID] = -1;
    }
    else
    {
        m_kbd_mtx[0].m_alts_row[m_cur_HID - 0x100] = -1;
        m_kbd_mtx[0].m_alts_col[m_cur_HID - 0x100] = -1;
    }
    update_ui();
}

void MainWindow::on_actionOpen_triggered()
{
    m_file = QFileDialog::getOpenFileName(this, tr("Укажите файл с раскладкой"), "", filter, &filter);
    if (m_file.length() > 0)
    {
        QFile f(m_file);
        if (f.open(QIODevice::ReadOnly))
        {
            QByteArray json = f.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(json);
            QJsonObject root = doc.object();
            for (int i=0 ; i<2 ; ++i)
            {
                QJsonObject mtx = root[QString::number(i)].toObject();
                for (int j=0 ; j<128 ; ++j)
                {
                    if (mtx.contains(QString::number(j)))
                    {
                        QJsonObject node = mtx[QString::number(j)].toObject();
                        m_kbd_mtx[i].m_keys_row[j] = node["r"].toInt();
                        m_kbd_mtx[i].m_keys_col[j] = node["c"].toInt();
                    }
                    if (mtx.contains(QString::number(0x100 + j)))
                    {
                        QJsonObject node = mtx[QString::number(0x100 + j)].toObject();
                        m_kbd_mtx[0].m_alts_row[j] = node["r"].toInt();
                        m_kbd_mtx[0].m_alts_col[j] = node["c"].toInt();
                    }
                }
            }
            on_actionLat_triggered();
        }
        f.close();
    }
}

void MainWindow::on_actionSave_triggered()
{
    save_to_file();
}

void MainWindow::on_actionSave_as_triggered()
{
    m_file = QFileDialog::getSaveFileName(this, tr("Укажите файл с раскладкой"), "", filter, &filter);
    if (m_file.length() > 0)
        save_to_file();
}

void MainWindow::on_actionLat_triggered()
{
    m_cur_mtx = 0;
    ui->actionLat->setChecked(true);
    ui->actionCyr->setChecked(false);
    update_ui();
}

void MainWindow::on_actionCyr_triggered()
{
    m_cur_mtx = 1;
    ui->actionLat->setChecked(false);
    ui->actionCyr->setChecked(true);
    update_ui();
}

void MainWindow::on_actionNew_triggered()
{
    for (int i=0 ; i<128 ; ++i)
    {
        m_kbd_mtx[0].m_keys_row[i] = -1;
        m_kbd_mtx[0].m_keys_col[i] = -1;
        m_kbd_mtx[0].m_alts_row[i] = -1;
        m_kbd_mtx[0].m_alts_col[i] = -1;

        m_kbd_mtx[1].m_keys_row[i] = -1;
        m_kbd_mtx[1].m_keys_col[i] = -1;
        m_kbd_mtx[1].m_alts_row[i] = -1;
        m_kbd_mtx[1].m_alts_col[i] = -1;
    }

    on_actionLat_triggered();
}

#define TO_MTX(arr_row, arr_col, idx) \
    int row = arr_row[idx]; \
    int col = arr_col[idx]; \
    if ((row == -1) || (col == -1)) \
    { \
        row = 15; \
        col = 15; \
    } \
    ln += "KMX(" + QString::number(row) + "," + \
            QString::number(col) + "),";

#define SAVE_MTX(idx, kname) \
    lst.push_back(QString("const uint8_t kbd_" + name + "_" + kname + "[128] = {")); \
    for (int i=0 ; i<16 ; ++i) \
    { \
        ln = "\t\t\t\t\t\t"; \
        for (int j=0 ; j<8 ; ++j) \
        { \
            TO_MTX(m_kbd_mtx[idx].m_keys_row, m_kbd_mtx[idx].m_keys_col, i*8+j) \
        } \
        if (i == 15) \
        { \
            ln.remove(ln.size() - 1, 1); \
            ln += "};"; \
        } \
        lst.push_back(ln); \
    }

void MainWindow::on_actionExport_header_triggered()
{
    if (m_file.length() == 0)
        return;

    QStringList fns = m_file.split("/");
    QString path = m_file.left(m_file.lastIndexOf('/') + 1);
    QString name = fns[fns.size() - 1];
    name = name.left(name.lastIndexOf('.'));

    QStringList lst;
    QString ln;

    lst.push_back(QString("#ifndef _KBD_" + name.toUpper() + "_DATA_H_"));
    lst.push_back(QString("#define _KBD_" + name.toUpper() + "_DATA_H_"));
    lst.push_back("");
    lst.push_back("#include <inttypes.h>");
    lst.push_back("");
    SAVE_MTX(0, "lat");
    SAVE_MTX(1, "rus");
    ln = "const uint8_t kbd_" + name + "_f[8] = {";
    for (int i=0 ; i<8 ; ++i)
    {
        TO_MTX(m_kbd_mtx[0].m_alts_row, m_kbd_mtx[0].m_alts_col, (m_alts[i] - 0x100))
    }
    ln.remove(ln.size() - 1, 1);
    ln += "};";
    lst.push_back(ln);

    lst.push_back(QString("kbd_matrix_t kbd_rk86 = {kbd_" + name + "_lat, kbd_" + name + "_rus, kbd_" + name + "_f};"));
    lst.push_back("");
    lst.push_back("#endif");
    lst.push_back("");

    //name = m_file.left(m_file.lastIndexOf('.')) + ".h";
    QFile f(path + "kbd_mtx_" + name + ".h");
    if (f.open(QIODevice::WriteOnly))
        for (int i=0 ; i<lst.length() ; ++i)
            f.write((lst[i] + "\n").toLatin1());
    f.close();
}

void MainWindow::save_to_file()
{
    if (m_file.length() == 0)
        return;

    QJsonDocument doc;
    QJsonObject root;
    for (int i=0 ; i<2 ; ++i)
    {
        QJsonObject mtx;
        for (int j=0 ; j<128 ; ++j)
        {
            if (m_kbd_mtx[i].m_keys_row[j] != -1)
            {
                QJsonObject node;
                node["r"] = m_kbd_mtx[i].m_keys_row[j];
                node["c"] = m_kbd_mtx[i].m_keys_col[j];
                mtx[QString::number(j)] = node;
            }
            if (m_kbd_mtx[0].m_alts_row[j] != -1)
            {
                QJsonObject node;
                node["r"] = m_kbd_mtx[0].m_alts_row[j];
                node["c"] = m_kbd_mtx[0].m_alts_col[j];
                mtx[QString::number(j + 0x100)] = node;
            }
        }
        root[QString::number(i)] = mtx;
    }
    doc.setObject(root);
    QByteArray json = doc.toJson();

    QFile f(m_file);
    if (f.open(QIODevice::WriteOnly))
        f.write(json);
    f.close();
}

void MainWindow::update_ui()
{
    for (int i=0 ; i<128 ; ++i)
    {
        QAbstractButton* btn = m_buttons_container->button(i);
        int row = m_kbd_mtx[m_cur_mtx].m_keys_row[i];
        int col = m_kbd_mtx[m_cur_mtx].m_keys_col[i];
        if (btn != nullptr)
        {
            if (row != -1)
            {
                btn->setStyleSheet("background-color: DarkGreen");
                btn->setText(m_strs[i] + "[" + QString::number(row) + ":" + QString::number(col) + "]");
            }
            else
            {
                btn->setStyleSheet("");
                btn->setText(m_strs[i]);
            }
        }

        btn = m_buttons_container->button(0x100 + i);
        row = m_kbd_mtx[0].m_alts_row[i];
        col = m_kbd_mtx[0].m_alts_col[i];
        if (btn != nullptr)
        {
            if (row != -1)
            {
                btn->setStyleSheet("background-color: DarkGreen");
                btn->setText(m_strs[0x100 + i] + "[" + QString::number(row) + ":" + QString::number(col) + "]");
            }
            else
            {
                btn->setStyleSheet("");
                btn->setText(m_strs[0x100 + i]);
            }
        }
    }
}

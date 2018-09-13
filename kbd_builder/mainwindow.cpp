#include "mainwindow.h"
#include "ui_mainwindow.h"

#define KEY_ROWS 6
#define KEY_COLS 21
int keys_code[KEY_ROWS][KEY_COLS] =
    {{0x29,   -1, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,   -1,   -1,   -1,   -1},
     {0x35, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x2d, 0x2e, 0x2a, 0x49, 0x4a, 0x4b, 0x53, 0x54, 0x55, 0x56},
     {0x2b, 0x14, 0x1a, 0x08, 0x15, 0x17, 0x1c, 0x18, 0x0c, 0x12, 0x13, 0x2f, 0x30,   -1, 0x4c, 0x4d, 0x4e, 0x5f, 0x60, 0x61, 0x57},
     {0x39, 0x04, 0x16, 0x07, 0x09, 0x0a, 0x0b, 0x0d, 0x0e, 0x0f, 0x33, 0x34, 0x31, 0x28,   -1,   -1,   -1, 0x5c, 0x5d, 0x5e,   -1},
     {0x102,0x1d, 0x1b, 0x06, 0x19, 0x05, 0x11, 0x10, 0x35, 0x37, 0x38,   -1,   -1,0x120,   -1, 0x52,   -1, 0x59, 0x5a, 0x5b, 0x58},
     {0x101,0x108,0x104,  -1,   -1,   -1,   -1, 0x2c,   -1,   -1,   -1,0x140,0x180,0x110, 0x50, 0x51, 0x4f, 0x62,   -1, 0x63,   -1}};
QString keys_str[KEY_ROWS][KEY_COLS] =
    {{"Esc", "", "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12", "PrnScr", "ScrlLock", "Pause", "", "", "", ""},
     {"`~", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "=", "Bcksp", "Insert", "Home", "PgUp", "NumLock", "/", "*", "-"},
     {"Tab", "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "[", "]", "", "Del", "End", "PgDown", "7", "8", "9", "+"},
     {"CapsLock", "A", "S", "D", "F", "G", "H", "J", "K", "L", ";", "'", "\\", "Enter", "", "", "", "4", "5", "6", ""},
     {"LShift", "Z", "X", "C", "V", "B", "N", "M", ",", ".", "/", "", "", "RShift", "", "Up", "", "1", "2", "3", "Enter"},
     {"LCtrl", "LGUI", "LAlt", "", "", "", "", "Space", "", "", "", "RAlt", "RGUI", "RCtrl", "Left", "Down", "Right", "0", "", ".", ""}};

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

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

    for (int i=0 ; i<256 ; ++i)
    {
        m_keys_row[i] = -1;
        m_keys_col[i] = -1;
        m_alts_row[i] = -1;
        m_alts_col[i] = -1;
    }

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
        m_keys_row[m_cur_HID] = cc.first;
        m_keys_col[m_cur_HID] = cc.second;
    }
    else
    {
        m_alts_row[m_cur_HID - 0x100] = cc.first;
        m_alts_col[m_cur_HID - 0x100] = cc.second;
    }
    m_cur_btn->setStyleSheet("background-color: DarkGreen");
}

void MainWindow::cell_clear(bool fl)
{
    (void)(fl);
    if (m_cur_HID < 0x100)
    {
        m_keys_row[m_cur_HID] = -1;
        m_keys_col[m_cur_HID] = -1;
    }
    else
    {
        m_alts_row[m_cur_HID - 0x100] = -1;
        m_alts_col[m_cur_HID - 0x100] = -1;
    }
    m_cur_btn->setStyleSheet("");
}

void MainWindow::on_actionOpen_triggered()
{
    //
}

void MainWindow::on_actionSave_triggered()
{
    //
}

void MainWindow::on_actionSave_as_triggered()
{
    //
}

#include <QFile> 
#include "crc32.h"

int main(int argc, char** argv)
{
    if (argc != 3)
        return 1;
    Crc32 crc;
    uint32_t crc32 = crc.calculateFromFile(argv[1]);

    QFile f(QString(argv[1]) + ".sign");
    if (f.open(QIODevice::WriteOnly))
    {
        uint32_t version = QString(argv[2]).toInt();
        f.write((const char*)&version, sizeof(uint32_t));
        f.write((const char*)&crc32, sizeof(uint32_t));
    }
    f.close();

    return 0;
}

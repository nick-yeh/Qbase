#include "Qbase.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Qbase w;
	w.show();
	return a.exec();
}

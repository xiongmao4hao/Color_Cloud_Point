#include "SkeletonDataViewer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	SkeletonDataViewer w;
	w.show();
	return a.exec();
}

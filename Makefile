#############################################################################
# Makefile for building: ZBRy
# Generated by qmake (2.01a) (Qt 4.8.1) on: Wed Apr 16 21:47:53 2014
# Project:  ZBRy.pro
# Template: app
# Command: /usr/bin/qmake-qt4 -spec /usr/share/qt4/mkspecs/linux-g++ CONFIG+=debug CONFIG+=declarative_debug -o Makefile ZBRy.pro
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = -DQT_WEBKIT -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED
CFLAGS        = -pipe -g -Wall -W -D_REENTRANT $(DEFINES)
CXXFLAGS      = -pipe -g -Wall -W -D_REENTRANT $(DEFINES)
INCPATH       = -I/usr/share/qt4/mkspecs/linux-g++ -I. -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I. -I. -I.
LINK          = g++
LFLAGS        = 
LIBS          = $(SUBLIBS)  -L/usr/lib/x86_64-linux-gnu -lQtGui -lQtCore -lpthread 
AR            = ar cqs
RANLIB        = 
QMAKE         = /usr/bin/qmake-qt4
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = kinematicpoints.cpp \
		main.cpp \
		mainwindow.cpp \
		trajectorypoints.cpp \
		widgetanimation.cpp \
		widgetsettings.cpp moc_kinematicpoints.cpp \
		moc_mainwindow.cpp \
		moc_trajectorypoints.cpp \
		moc_widgetanimation.cpp \
		moc_widgetsettings.cpp
OBJECTS       = kinematicpoints.o \
		main.o \
		mainwindow.o \
		trajectorypoints.o \
		widgetanimation.o \
		widgetsettings.o \
		moc_kinematicpoints.o \
		moc_mainwindow.o \
		moc_trajectorypoints.o \
		moc_widgetanimation.o \
		moc_widgetsettings.o
DIST          = /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/modules/qt_webkit_version.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/debug.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/declarative_debug.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		ZBRy.pro
QMAKE_TARGET  = ZBRy
DESTDIR       = 
TARGET        = ZBRy

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET): ui_mainwindow.h ui_widgetanimation.h ui_widgetsettings.h $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)
	{ test -n "$(DESTDIR)" && DESTDIR="$(DESTDIR)" || DESTDIR=.; } && test $$(gdb --version | sed -e 's,[^0-9]\+\([0-9]\)\.\([0-9]\).*,\1\2,;q') -gt 72 && gdb --nx --batch --quiet -ex 'set confirm off' -ex "save gdb-index $$DESTDIR" -ex quit '$(TARGET)' && test -f $(TARGET).gdb-index && objcopy --add-section '.gdb_index=$(TARGET).gdb-index' --set-section-flags '.gdb_index=readonly' '$(TARGET)' '$(TARGET)' && rm -f $(TARGET).gdb-index || true

Makefile: ZBRy.pro  /usr/share/qt4/mkspecs/linux-g++/qmake.conf /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/modules/qt_webkit_version.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/debug.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/declarative_debug.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		/usr/lib/x86_64-linux-gnu/libQtGui.prl \
		/usr/lib/x86_64-linux-gnu/libQtCore.prl
	$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++ CONFIG+=debug CONFIG+=declarative_debug -o Makefile ZBRy.pro
/usr/share/qt4/mkspecs/common/unix.conf:
/usr/share/qt4/mkspecs/common/linux.conf:
/usr/share/qt4/mkspecs/common/gcc-base.conf:
/usr/share/qt4/mkspecs/common/gcc-base-unix.conf:
/usr/share/qt4/mkspecs/common/g++-base.conf:
/usr/share/qt4/mkspecs/common/g++-unix.conf:
/usr/share/qt4/mkspecs/qconfig.pri:
/usr/share/qt4/mkspecs/modules/qt_webkit_version.pri:
/usr/share/qt4/mkspecs/features/qt_functions.prf:
/usr/share/qt4/mkspecs/features/qt_config.prf:
/usr/share/qt4/mkspecs/features/exclusive_builds.prf:
/usr/share/qt4/mkspecs/features/default_pre.prf:
/usr/share/qt4/mkspecs/features/debug.prf:
/usr/share/qt4/mkspecs/features/default_post.prf:
/usr/share/qt4/mkspecs/features/declarative_debug.prf:
/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf:
/usr/share/qt4/mkspecs/features/warn_on.prf:
/usr/share/qt4/mkspecs/features/qt.prf:
/usr/share/qt4/mkspecs/features/unix/thread.prf:
/usr/share/qt4/mkspecs/features/moc.prf:
/usr/share/qt4/mkspecs/features/resources.prf:
/usr/share/qt4/mkspecs/features/uic.prf:
/usr/share/qt4/mkspecs/features/yacc.prf:
/usr/share/qt4/mkspecs/features/lex.prf:
/usr/share/qt4/mkspecs/features/include_source_dir.prf:
/usr/lib/x86_64-linux-gnu/libQtGui.prl:
/usr/lib/x86_64-linux-gnu/libQtCore.prl:
qmake:  FORCE
	@$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++ CONFIG+=debug CONFIG+=declarative_debug -o Makefile ZBRy.pro

dist: 
	@$(CHK_DIR_EXISTS) .tmp/ZBRy1.0.0 || $(MKDIR) .tmp/ZBRy1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) .tmp/ZBRy1.0.0/ && $(COPY_FILE) --parents kinematicpoints.h mainwindow.h trajectorypoints.h widgetanimation.h widgetsettings.h .tmp/ZBRy1.0.0/ && $(COPY_FILE) --parents kinematicpoints.cpp main.cpp mainwindow.cpp trajectorypoints.cpp widgetanimation.cpp widgetsettings.cpp .tmp/ZBRy1.0.0/ && $(COPY_FILE) --parents mainwindow.ui widgetanimation.ui widgetsettings.ui .tmp/ZBRy1.0.0/ && (cd `dirname .tmp/ZBRy1.0.0` && $(TAR) ZBRy1.0.0.tar ZBRy1.0.0 && $(COMPRESS) ZBRy1.0.0.tar) && $(MOVE) `dirname .tmp/ZBRy1.0.0`/ZBRy1.0.0.tar.gz . && $(DEL_FILE) -r .tmp/ZBRy1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


check: first

mocclean: compiler_moc_header_clean compiler_moc_source_clean

mocables: compiler_moc_header_make_all compiler_moc_source_make_all

compiler_moc_header_make_all: moc_kinematicpoints.cpp moc_mainwindow.cpp moc_trajectorypoints.cpp moc_widgetanimation.cpp moc_widgetsettings.cpp
compiler_moc_header_clean:
	-$(DEL_FILE) moc_kinematicpoints.cpp moc_mainwindow.cpp moc_trajectorypoints.cpp moc_widgetanimation.cpp moc_widgetsettings.cpp
moc_kinematicpoints.cpp: kinematicpoints.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) kinematicpoints.h -o moc_kinematicpoints.cpp

moc_mainwindow.cpp: widgetanimation.h \
		trajectorypoints.h \
		kinematicpoints.h \
		widgetsettings.h \
		mainwindow.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) mainwindow.h -o moc_mainwindow.cpp

moc_trajectorypoints.cpp: trajectorypoints.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) trajectorypoints.h -o moc_trajectorypoints.cpp

moc_widgetanimation.cpp: trajectorypoints.h \
		kinematicpoints.h \
		widgetanimation.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) widgetanimation.h -o moc_widgetanimation.cpp

moc_widgetsettings.cpp: kinematicpoints.h \
		widgetsettings.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) widgetsettings.h -o moc_widgetsettings.cpp

compiler_rcc_make_all:
compiler_rcc_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_moc_source_make_all:
compiler_moc_source_clean:
compiler_uic_make_all: ui_mainwindow.h ui_widgetanimation.h ui_widgetsettings.h
compiler_uic_clean:
	-$(DEL_FILE) ui_mainwindow.h ui_widgetanimation.h ui_widgetsettings.h
ui_mainwindow.h: mainwindow.ui
	/usr/bin/uic-qt4 mainwindow.ui -o ui_mainwindow.h

ui_widgetanimation.h: widgetanimation.ui
	/usr/bin/uic-qt4 widgetanimation.ui -o ui_widgetanimation.h

ui_widgetsettings.h: widgetsettings.ui
	/usr/bin/uic-qt4 widgetsettings.ui -o ui_widgetsettings.h

compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: compiler_moc_header_clean compiler_uic_clean 

####### Compile

kinematicpoints.o: kinematicpoints.cpp kinematicpoints.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o kinematicpoints.o kinematicpoints.cpp

main.o: main.cpp mainwindow.h \
		widgetanimation.h \
		trajectorypoints.h \
		kinematicpoints.h \
		widgetsettings.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o main.o main.cpp

mainwindow.o: mainwindow.cpp mainwindow.h \
		widgetanimation.h \
		trajectorypoints.h \
		kinematicpoints.h \
		widgetsettings.h \
		ui_mainwindow.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o mainwindow.o mainwindow.cpp

trajectorypoints.o: trajectorypoints.cpp trajectorypoints.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o trajectorypoints.o trajectorypoints.cpp

widgetanimation.o: widgetanimation.cpp widgetanimation.h \
		trajectorypoints.h \
		kinematicpoints.h \
		ui_widgetanimation.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o widgetanimation.o widgetanimation.cpp

widgetsettings.o: widgetsettings.cpp widgetsettings.h \
		kinematicpoints.h \
		ui_widgetsettings.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o widgetsettings.o widgetsettings.cpp

moc_kinematicpoints.o: moc_kinematicpoints.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_kinematicpoints.o moc_kinematicpoints.cpp

moc_mainwindow.o: moc_mainwindow.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_mainwindow.o moc_mainwindow.cpp

moc_trajectorypoints.o: moc_trajectorypoints.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_trajectorypoints.o moc_trajectorypoints.cpp

moc_widgetanimation.o: moc_widgetanimation.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_widgetanimation.o moc_widgetanimation.cpp

moc_widgetsettings.o: moc_widgetsettings.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_widgetsettings.o moc_widgetsettings.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:


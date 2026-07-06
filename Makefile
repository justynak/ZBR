# Makefile for ZBRy — 5-DOF robot simulation (Qt 5 Widgets)
#
# Targets:
#   make          build ./ZBRy
#   make clean    remove build artifacts
#   make run      build and launch

TARGET   := ZBRy
BUILDDIR := build

CXX      ?= g++
MOC      ?= moc
UIC      ?= uic

SOURCES     := main.cpp mainwindow.cpp widgetanimation.cpp widgetsettings.cpp \
               kinematicpoints.cpp trajectorypoints.cpp
MOC_HEADERS := mainwindow.h widgetanimation.h widgetsettings.h \
               kinematicpoints.h trajectorypoints.h
FORMS       := mainwindow.ui widgetanimation.ui widgetsettings.ui

# Qt flags via pkg-config when available; otherwise fall back to the
# standard Debian/Ubuntu multiarch layout.
QT_CFLAGS := $(shell pkg-config --cflags Qt5Widgets 2>/dev/null)
QT_LIBS   := $(shell pkg-config --libs Qt5Widgets 2>/dev/null)
ifeq ($(strip $(QT_CFLAGS)),)
  QT_INC    := /usr/include/x86_64-linux-gnu/qt5
  QT_CFLAGS := -I$(QT_INC) -I$(QT_INC)/QtWidgets -I$(QT_INC)/QtGui -I$(QT_INC)/QtCore
  QT_LIBS   := -lQt5Widgets -lQt5Gui -lQt5Core
endif

CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++17 -Wall -Wextra -fPIC $(QT_CFLAGS) -I. -I$(BUILDDIR)
LDLIBS   := $(QT_LIBS)

UI_HEADERS := $(FORMS:%.ui=$(BUILDDIR)/ui_%.h)
MOC_SRCS   := $(MOC_HEADERS:%.h=$(BUILDDIR)/moc_%.cpp)
OBJECTS    := $(SOURCES:%.cpp=$(BUILDDIR)/%.o) $(MOC_SRCS:%.cpp=%.o)
DEPS       := $(OBJECTS:%.o=%.d)

.PHONY: all clean run
.SECONDARY: $(MOC_SRCS) $(UI_HEADERS)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILDDIR)/%.o: %.cpp | $(BUILDDIR) $(UI_HEADERS)
	$(CXX) $(CXXFLAGS) -MMD -MP -c -o $@ $<

$(BUILDDIR)/%.o: $(BUILDDIR)/%.cpp | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -MMD -MP -c -o $@ $<

$(BUILDDIR)/moc_%.cpp: %.h | $(BUILDDIR)
	$(MOC) $(QT_CFLAGS) $< -o $@

$(BUILDDIR)/ui_%.h: %.ui | $(BUILDDIR)
	$(UIC) $< -o $@

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -rf $(BUILDDIR) $(TARGET)

-include $(DEPS)

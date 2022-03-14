/****************************************************************************
 * Copyright ©2017 Brian Curless.  All rights reserved.  Permission is hereby
 * granted to students registered for University of Washington CSE 457 or CSE
 * 557 for use solely during Autumn Quarter 2017 for purposes of the course.
 * No other use, copying, distribution, or modification is permitted without
 * prior written consent. Copyrights for third-party components of this work
 * must be honored.  Instructors interested in reusing these course materials
 * should contact the author.
 ****************************************************************************/
#ifndef BILATERALGAUSSDIALOG_H
#define BILATERALGAUSSDIALOG_H

#include <memory>
#include <QDialog>

class PaintView;

namespace Ui {
    class FilterDialog;
}

class FilterDialog : public QDialog {
    Q_OBJECT
public:
    explicit FilterDialog(QWidget *parent = 0);
    ~FilterDialog();
    virtual int exec() override;
    double a() const { return a_; }
    unsigned int Iterations() const { return iterations_; }
private:
    Ui::FilterDialog *ui;
    double a_;
    unsigned int iterations_;
};

#endif // BILATERALGAUSSDIALOG_H

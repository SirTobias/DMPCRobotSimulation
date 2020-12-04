        #ifndef INTERSECTIONCELL_H
    #define INTERSECTIONCELL_H
    #include <QtCore/QVector>
    #include <reservationcell.h>
    #include <memory>

    /**
     * @brief The InterSectionCell class holds the reserved times for each cars
     * //TODO: probably this class derived from SimResource (?)
     */
    class InterSectionCell
    {
    public:
        InterSectionCell(const unsigned int &x, const unsigned int &y);
        bool isTimeReserved(const double &time) const;
        bool isPrelimTimeReserved(const double &time) const;
        bool reserveTimeForCar(const QString &car, const double &time);
        double reserveNextFreeTimeForCar(const QString& car, const double &time);
        bool removeCar(const QString &car);
        double getNextFreeTime(const double& start = 0.0) const;
        double getNextFreePrelimTime(const double &start = 0.0) const;
        bool reservePremlimNextTime(const QString& car, const double& time);
        bool isTimeAlreadyReserved(const QString& car, const double& time) const;
        bool isPrelimTimeAlreadyReserved(const QString& car, const double& time) const;
        double getTimeForCar(const QString& car, const double& t) const;
        double getPrelimTimeForCar(const QString& car, const double& t) const;
        void removePrelimPathFromCar(const QString& car);
        bool operator==(const std::shared_ptr<InterSectionCell> compCell) const;
        //bool operator<(const std::shared_ptr<InterSectionCell> compCell) const;
        unsigned int replacePrelimWithReservedPositions();
        unsigned int getNumOfReservations();

        double getX() const;
        double getY() const;
        const QVector<ReservationCell> &getReservationCells() const;
        const QVector<ReservationCell>& getPrelimReservationCells() const;



        double getTentativeGCost() const;
        void setTentativeGCost(double value);

        double getFCost() const;
        void setFCost(double value);

        std::weak_ptr<InterSectionCell> getParent() const;
        void setParent(const std::weak_ptr<InterSectionCell> &value);
        //bool compare (const std::shared_ptr<InterSectionCell> left, const std::shared_ptr<InterSectionCell> right)const;


       /* struct compare{
            bool operator()(const std::shared_ptr<InterSectionCell> left, const std::shared_ptr<InterSectionCell> right)const{
                return (*left) < (*right) ;
            }
        };*/


        /*double getRhs() const;
        void setRhs(double value);



        double getG() const;
        void setG(double value);

        double getKey() const;
        void setKey(double value);*/

    private:
        ///the time vector which holds the reserved time for the cell
        QVector<ReservationCell> m_reserved;
        ///preliminary reserved time
        QVector<ReservationCell> m_prelimReserved;
        ///index for x
        unsigned int m_x;
        ///index for y
        unsigned int m_y;
        double tentativeGCost; // astar
        double fCost; //astar
        std::weak_ptr<InterSectionCell> parent; //astar

        //double rhs; //dstar
        //double g; //dstar
        //double key; //dstar

    };

    #endif // INTERSECTIONCELL_H

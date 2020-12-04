#include "databasecore.h"


DataBaseCore::DataBaseCore(const QString driver){
    m_db = new QSqlDatabase( QSqlDatabase::addDatabase(driver) );
}

DataBaseCore::DataBaseCore(){
    m_db = new QSqlDatabase( QSqlDatabase::addDatabase("QPSQL") );
}


DataBaseCore::~DataBaseCore(){
    qDebug() << "Called Destructor!";
    delete m_db;
}

bool DataBaseCore::connect( const QString& server,
                                     const QString& databaseName,
                                     const QString& userName,
                                     const QString& password )
{
    m_db->setConnectOptions();
    m_db->setHostName(server);
    m_db->setDatabaseName(databaseName);
    m_db->setUserName(userName);
    m_db->setPassword(password);


    if(m_db->open()) {
        m_query = new QSqlQuery(*m_db);
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief output data with a form of like table
 *
 */
void DataBaseCore::output(){

    QString out = "";

    for(int i = 0; i < m_query->record().count(); i++){
        out += m_query->record().fieldName(i) + " ";
    }
#if QT_VERSION >= 0x050400
    qDebug().noquote() << out;
#else
    qDebug() << out;
#endif
    do{
        out.clear();
        for(int i = 0; i< m_query->record().count(); i++){
            out += m_query->value(i).toString() + "  ";
        }
#if QT_VERSION >= 0x050400
        qDebug().noquote() << out;
#else
        qDebug() << out;
#endif

    }while(m_query->next());

    qDebug() << "The size of the result: " << m_query->size();

}

/**
 * @brief check variables, like(key-value),
 *
 * @param keys which means a list of columns of table
 * @param values  which means a list of values
 *
 * return true when both of keys and values are empty, and their size are the same when both of keys and values are not empty
 *
 */
bool DataBaseCore::CheckParams(const QStringList& keys, const  QVariantList& values){

    if((keys.isEmpty() && !values.isEmpty()) || (!keys.isEmpty() && values.isEmpty())){
        //return false when one of both keys and values is empty
        return false;
    }else if( !keys.isEmpty() && !values.isEmpty() && keys.size() != values.size()){
        //return false when two parameters both are not empty and their size are not the same.
        return false;
    }

    return true;
}

/**
 * @brief creat SQL statement for operatins query
 *
 * @param tables  which means names of multiple tables
 * @param columns which means all columns of multiple tables
 *
 *
 */
QString DataBaseCore::CreateQuery(const QStringList& tables, const QStringList& columns){

    QString str = "SELECT * FROM ";

    if (tables.contains(t_Car) && tables.contains(t_Path) && tables.contains(t_PathItem)){

        //str += "car,path,pathitem WHERE car.carid = path.carid AND path.id = pathitem.pathid ";
        str += t_Car + "," + t_Path + "," + t_PathItem + " WHERE " +
                t_Car + "." + p_CarId + " = " + t_Path + "." +p_CarId + " AND " +
                t_Path + "." + p_PathId + " = " + t_PathItem + "." + p_PathId + " ";

    }else if (tables.contains(t_Car) && tables.contains(t_Path) && !tables.contains(t_PathItem)){

        //str += "car,path WHERE car.carid = path.carid ";
        str += t_Car + "," + t_Path + " WHERE " +
                t_Car + "." + p_CarId + " = " + t_Path + "." + p_CarId + " ";

    }else if (tables.contains(t_PathItem) && tables.contains(t_Path) && !tables.contains(t_Car)){

        //str += "path,pathitem WHERE path.id = pathitem.pathid ";
        str += t_Path + "," + t_PathItem + " WHERE " +
                t_Path + "." + p_PathId + " = " + t_PathItem + "." + p_PathId + " ";

    }else if ((tables.contains(t_Car) || tables.contains(t_Path) || tables.contains(t_PathItem)) && tables.size() == 1){ //Default table = 1;

        if (!columns.isEmpty()){
            str += tables.value(0) + " WHERE ";
        }else
            str += tables.value(0);

    }else{
        qDebug() << "There doesn't exist the relation you are looking for in database. Please check it";
    }
    return str;
}

/**
 * @brief select data referring to the range of giving values from multiple tables
 *
 * @param tables  which means names of multiple tables
 * @param columns which means all columns of multiple tables
 * @param values      which means values of the given columns
 * @param relations   which means the relation between a column and a value, such as ">", "<", "<=", ">=" and "=".
 *
 */
int DataBaseCore::Query(const QStringList& tables, const QStringList& columns, const QVariantList& values, const QStringList& relations){

    if(!CheckParams(columns, values))
        return -2;

    QString str = CreateQuery(tables, columns);

    QString out = "";

    if(!columns.isEmpty()){
        if(tables.size() != 1){
            out = " AND ";
        }
        for(int i = 0; i < columns.size(); i++){
            if(columns.value(i) == p_CarId){
                out += t_Car + "." + columns.value(i);
            }else if( columns.value(i) == p_PathId){
                out += t_Path + "." + columns.value(i);
            }else
                out += columns.value(i);

            if(relations.at(i).isEmpty()){
                out += " = '" + values.value(i).toString() + "'";
            }else
                out += relations.value(i) + "'" + values.value(i).toString() + "'";

            if(i < columns.size()-1){
                out += " AND ";
            }
        }
        str += out;
    }
//    qDebug() << str;
    if(!m_query->prepare(str)){
        qDebug() << m_db->lastError().text();
        return -3;
    }else{
        if(Execute()){
            output();
            return 1; //m_query->size();
        }
    }

    return 0;

}

/**
 * @brief select data from multiple tables
 *
 * @param tables which means names of multiple tables
 * @param columns which means coloumns of tables
 * @param values which means values of these given columns
 *
 */
int DataBaseCore::Query(const QStringList& tables, const QStringList& columns, const QVariantList& values){

    if(!CheckParams(columns, values))
        return -1;

    QString str = CreateQuery(tables, columns);
//    qDebug() << "str= " << str;
    QString out = "";

    if(!columns.isEmpty()){
        if(tables.size() > 1){
            out = "AND ";
        }
        for(int i = 0; i < columns.size(); i++){
            if (columns.value(i) == p_CarId){
                out += t_Car + "." + columns.value(i) + " = '" + values.value(i).toString() + "'";
            }else if(columns.value(i) == p_PathId){
                out += t_Path + "." + columns.value(i) + " = '" + values.value(i).toString() + "'";
            }else{
                out += columns.value(i) + " = '" + values.value(i).toString() + "'";
//                qDebug() << out;
            }

            if(i < columns.size()-1){
                out += " AND ";
            }
        }
        str += out;
    }
    qDebug() << str;
    if(!m_query->prepare(str)){
        qDebug() << m_db->lastError().text();
        return -2;
    }else{
        if(Execute()){
            output();
            return 1; //m_query->size();
        }
    }

    return 0;
}


/**
 * @brief select data from a single table
 *
 * @param table     which means name of table
 * @param column    which means name of column
 * @param value     whcih means value of the given column
 * @param relation  which means the relation between a column and a value, such as ">", "<", "<=", ">=" and "=".
 *
 */
int DataBaseCore::Query(const QString& table, const QString& column, const QVariant& value, const QString& relation){

    if( (!column.isEmpty() && value.toString().isEmpty()) || (column.isEmpty() && !value.toString().isEmpty())){
        //return -1 when one of both column and value is empty
        return -2;
    }

    QString str = "SELECT * FROM table WHERE column = ?";
//    qDebug() << "relation = " << relation;
    if(column.isNull()){
        str = "SELECT * FROM table";
    }else
        str = str.replace("column",column);
        str = str.replace("=", relation);

    str = str.replace("table", table);
//    qDebug() << str;
    if( !m_query->prepare(str) ){
        qDebug() << "Error, " << m_db->lastError().text();
        return -3;
    }else{
        m_query->addBindValue(value);
        if(Execute()){
            output();
            return 1; // m_query->size();
        }
    }
    return 0;

}

/**
 * @brief delete all data from a table
 *
 * @param table    which means name of table
 *
 */
int DataBaseCore::Delete(const QString &table){

    QString str = "DELETE FROM table ";
    str = str.replace("table",table);

    if(!m_query->prepare(str)){
        qDebug() << "Error = " << m_db->lastError().text();
        return -1;
    }

    if( Execute() )
        qDebug() << "Deleted " << m_query->numRowsAffected() << "records Successfully";
    else{
        qDebug() << "Failed to delete operation";
        return -2;
    }

    return 1; //m_query->numRowsAffected();
}

/**
 * @brief delete data from a single data
 *
 * @param table    which means name of table
 * @param columns   which means name of columns of the table
 * @param values    which means values of these given columns
 *
 */
int DataBaseCore::Delete(const QString &table, const QStringList &columns, const QVariantList &values){

    if(!CheckParams(columns, values))
        return -1;

    QString str = "DELETE  FROM table ";
    str = str.replace("table",table);


    if(!columns.isEmpty()){
        str += " WHERE ";
        for(int i = 0; i < columns.size(); i++){
            str += columns.value(i) + " = ?";
            if(columns.size() != 1 && i < columns.size() - 1)
                str += " AND ";
        }
    }

    if( !m_query->prepare(str) ){
        qDebug() << "Error = " << m_db->lastError().text();
        return -2;
    }else{
        for(int i = 0; i < columns.size(); i++)
            m_query->bindValue(i,values.value(i));

    }

    if( Execute() )
        qDebug() << "Deleted" << m_query->numRowsAffected() << "records successfully";
    else{
        qDebug() << "Failed to delete operation";
        return -3;
    }

    return 1; // m_query->numRowsAffected();

}

/**
 * @brief delete data referring to the range of giving values from a single table
 *
 * @param table    which means name of table
 * @param columns   which names of column of the table
 * @param values    which values of the given columns
 * @param relations    which means relation between every column and every valus, such as "<" ,">","<=",">=","=".
 *
 */
int DataBaseCore::Delete(const QString& table, const QStringList& columns, const QVariantList& values, const QStringList& relations){

    if(!CheckParams(columns, values))
        return -1;

    QString str = "DELETE FROM table ";
    str = str.replace("table",table);


    if(!columns.isEmpty()){
        str += " WHERE ";
        for(int i = 0; i < columns.size(); i++){
            if(relations.at(i).isEmpty()){
                str += columns.value(i) + " = ?";
            }else
                str += columns.value(i) + " " + relations.value(i) +" ? ";
            if(columns.size() != 1 && i < columns.size() - 1)
                str += " AND ";
        }
    }

    qDebug() << "STR = " << str;
    if( !m_query->prepare(str) ){
        qDebug() << "Error = " << m_db->lastError().text();
        return -2;
    }else{
        for(int i = 0; i < columns.size(); i++)
            m_query->bindValue(i,values.value(i));

    }

    if( Execute() )
        qDebug() << "Deleted" << m_query->numRowsAffected() << "records successfully";
    else{
        qDebug() << "Failed to delete operation";
        return -3;
    }

    return 1; // m_query->numRowsAffected();
}


/**
 * @brief delete a batch of data from a single table
 *
 * @param table    which means name of table
 * @param column    which means names of column
 * @param values    which means values of these given columns
 *
 */
int DataBaseCore::Delete_Batch(const QString& table, const QString& column, const QVariantList& values){

    if(column.isEmpty() || values.isEmpty()){
        qDebug() << "Any one of parameters can't be empty";
        return -1;
    }

    QString str = "DELETE FROM table WHERE column = ?";
    str = str.replace("table", table);
    str = str.replace("column", column);

    qDebug() << "STR = " << str;
    if(!m_query->prepare(str)){
        qDebug() << "Error = " << m_db->lastError().text();
        return -2;
    }else
        m_query->addBindValue(values);

    if(Execute_Batch()){
        qDebug() << "Deleted a batch of data successfully";
        return 1; //m_query->numRowsAffected();
    }else{
        qDebug() << "Failed to delete operation";
        return -3;
    }

}

/**
 * @brief updata data from a single table
 *
 * @param table    which means name of table
 * @param columns   which means names of columns whose value will be updated
 * @param newValues    which means new values of these given columns
 * @param conditinColumn    which means name of columns as condition of executing operation
 * @param conditionValue    which means value of these given conditionColumn
 *
 */
int DataBaseCore::Update(const QString& table, const QStringList& columns, const QVariantList& newValues, const QString& conditionColumn, const QVariant& conditionValue){
    //qDebug() << "Test";
    if(!CheckParams(columns, newValues))
        return -1;

    QString str = "UPDATE table set ";
    str = str.replace("table", table);

    for(int i = 0; i < columns.size(); i++){
        str += columns.value(i) + " = '" + newValues.value(i).toString() + "'";
        if(i < columns.size() - 1){
            str += " , ";
        }
    }
//    qDebug() << "id =" << conditionValue.toString();
    if(!conditionColumn.isEmpty())
        str += " WHERE " + conditionColumn + " = ? ";

    qDebug() << "STR = " << str;
    if( !m_query->prepare(str) )
    {
        qDebug() <<"Error = " << m_db->lastError().text();
        return -2;
    }else
        m_query->addBindValue(conditionValue);


    if( Execute() ){
        qDebug() << "Updated " << m_query->numRowsAffected() << " records Successfully";
    }else{
        qDebug() << "Failed to update operation";
        return -3;
    }

    return 1; //m_query->numRowsAffected();
}

/**
 * @brief insert data into a single table
 *
 * @param table    which means name of table
 * @param columns    which means names of columns
 * @param values    which means values of these given columns
 *
 */
int DataBaseCore::Insert(const QString& table, const QStringList& columns, const QVariantList& values){

    if(!CheckParams(columns, values))
        return -1;

    QString str ="INSERT INTO table( columns ) VALUES ( values )";
    str = str.replace("table",table);
    QString temp1 = "";
    QString temp2 = "";
    for(int i = 0; i < columns.size(); i++){
        temp1 += columns.value(i);
        temp2 += "'" + values.value(i).toString() + "'";
        if(i < columns.size() - 1){
            temp1 += ", ";
            temp2 += ", ";
        }
    }
    str = str.replace("columns", temp1);
    str = str.replace("values", temp2);

    qDebug() << "str = " << str;
    if( !m_query->prepare(str)){
        qDebug() <<"Error = " << m_db->lastError().text();
        return -2;
    }

    if( Execute() )
        qDebug() << "Inserted" << m_query->numRowsAffected() << " records Successfully";
    else{
        qDebug() << "Failed to insert operation";
        return -3;
    }

    return 1; //m_query->numRowsAffected();
}


/**
* @brief insert data into multiple tables
* @param tables which represents which tables will be operated
* @param columns which means the columns of tables that will be operated
* @param values which means values of the columns that will be inserted data into related tables
*
*/
int DataBaseCore::Insert(const QStringList& tables, const QStringList& columns, const QVariantList& values){
    if(!CheckParams(columns, values)){
        return -1;
    }
    m_db->transaction();
    QVariant temp = 0;

    for(int i = 0; i< tables.size(); i++){
        QString str = "INSERT INTO table( columns ) VALUES( values )";
        QString temp1 = "";
        QString temp2 = "";

        if( tables.value(i) == t_Car )
        {
            temp1 += columns.value(0);
            temp2 += "'" + values.value(0).toString() + "'";
            temp = 0;
        }
        else if ( tables.value(i) == t_Path )
        {
            if ( temp.toInt() != 0){
                temp1 += p_CarId;
                temp2 += "'" + temp.toString() + "'";
                temp = 0;
            }else{
                temp1 += columns.value(0);
                temp2 += "'" + values.value(0).toString() + "'";
            }


        }
        else if( tables.value(i) == t_PathItem )
        {
            if( tables.size() == 1 ){
                temp1 += columns.value(0);
                temp2 += "'" + values.value(0).toString() + "'";
            }else{
                temp1 += p_PathId + ", ";
                temp2 += "'" + temp.toString() + "', ";
                temp = 0;
            }

            for (int j = 1; j < values.size(); j++){
                temp1 += columns.value(j);
                temp2 += "'" + values.value(j).toString() + "'";

                if(j < columns.size()-1){
                    temp1 += ", ";
                    temp2 += ", ";
                }
            }

        }

//        qDebug() << temp1;
//        qDebug() << temp2;
        str = str.replace("table", tables.value(i));
        str = str.replace("columns", temp1);
        str = str.replace("values", temp2);

        if( !m_query->prepare(str)){
            qDebug() <<"Error = " << m_db->lastError().text();
//            return -1;
        }

        bool result = m_query->exec();
        temp = m_query->lastInsertId();
        if(m_query->lastError().type() != QSqlError::NoError || !result){
            qDebug() << "Operations occur errors: " << m_query->lastError().text();
            m_db->rollback();
            return -1;
        }
        temp1.clear();
        temp2.clear();
        str.clear();
    }
//    qDebug() << "End!";
    m_db->commit();
    return 1;
}


/**
 * @brief insert a batch of data into a table
 * @param table    which means name of table
 * @param columns    which means names of columns
 * @param listValues    which means a list of values of these given columns
 */
int DataBaseCore::Insert_Batch(const QString& table, const QStringList& columns, const QList<QVariantList>& listValues){

    if(columns.size() != listValues.size()){
        return -1;
    }

    QString str = "INSERT INTO table( columns ) VALUES( values )";
    str = str.replace("table",table);
    QString temp1 = "";
    QString temp2 = "";

    for(int i = 0; i < columns.size(); i++){
        temp1 += columns.at(i);
        temp2 += "?";
        if(i < columns.size() - 1){
            temp1 += ", ";
            temp2 += ", ";
        }
    }
    str = str.replace("columns",temp1);
    str = str.replace("values", temp2);

    if( !m_query->prepare(str)){
        qDebug() << "Error: " << m_db->lastError().text();
        return -2;
    }else{
        for(int i = 0; i < columns.size(); i++){
            m_query->addBindValue(listValues.value(i));
        }
    }

    if(Execute_Batch()){
        qDebug() << "Inserted " << m_query->numRowsAffected() << " records Successfully";
    }else{
        qDebug() << "Failed to Insert operation ";
        return -3;
    }

    return 1;

}


/**
 * @brief execute insert operation based on relation one to many among multiple tables(car, path, pathitem)
 * car,path,pathitem:(carname, state, time, y, x);
 * path,pathitem:(pathid, state, time, y, x);
 *
 * @param tables    which means names of tables being operated
 * @param lists    which means  a list of key-values like a column and a list of  values
 */
int DataBaseCore::Insert(QStringList& tables,const QMap<QString, QVariantList>& lists){
    m_db->transaction();
    QString str = "";
    QString constrains = ""; // constrains represents constrain when executing different insertions (path-pathitem / car-path-pathitem)
    QMap<QString, QVariant> records; //store inserted data that are unique in database

    if(tables.contains(t_Car) && tables.contains(t_Path) && tables.contains(t_PathItem)){
        constrains = p_CarName;
    }else if( tables.contains(t_Path) && tables.contains(t_PathItem)){
        constrains = p_CarId;
    }else{  //it means value of tables are not right.
        qDebug() << "Tables' names are not right";
        //return -1;
    }

    for(int i = 0; i< lists.first().size(); i++){
        QVariant temp = 0;
        char flag = false;
        for(int j = 0; j < tables.size(); j++){
            //if(tables.value(j) == "car"){ //car
            if(tables.value(j) == t_Car){
                flag = true;
                if (!records.contains(lists.value(constrains).value(i).toString())){
                    //str = "INSERT INTO car(carname) VALUES(:carname)";
                    str = " INSERT INTO " + t_Car + "(" + p_CarName + ") VALUES(:" + p_CarName+ ") ";
                    m_query->prepare(str);
                    //m_query.bindValue(":carid",lists.at(i).at(0));//carid
                    m_query->bindValue(":" + p_CarName, lists.value(p_CarName).value(i).toString());//carname
                }else{
                    continue;
                }
            }else if(tables.value(j) == t_Path){ //path
                if(!records.contains(lists.value(constrains).value(i).toString())){
                    //str = "INSERT INTO path(carid) VALUES(:carid)";
                    str = "INSERT INTO "+ t_Path + "(" + p_CarId + ") VALUES(:" + p_CarId + ")";
                    m_query->prepare(str);
                    if( flag ){
                        m_query->bindValue(":" + p_CarId, temp);
                        flag = false;
                    }else{
                        m_query->bindValue(":"+ p_CarId, lists.value(p_CarId).value(i).toString());
                    }
                }else{
                    continue;
                }
            }else if(tables.value(j) == t_PathItem){ // pathitem
                //str = "INSERT INTO pathitem(pathid, state, time, x, y) VALUES(:pathid, :state, :time, :x, :y)";
                str = "INSERT INTO " + t_PathItem + "(" + p_PathId+ ", " + p_State
                        + ", " + p_Time + ", " + p_X + ", " + p_Y + ") VALUES(:" + p_PathId
                        + ", :" + p_State + ", :" + p_Time + ", :" + p_X + ", :" + p_Y + ") ";
                m_query->prepare(str);
                m_query->bindValue(":" + p_PathId, records.find(lists.value(constrains).value(i).toString()).value());
                m_query->bindValue(":" + p_State, lists.value(p_State).value(i));
                m_query->bindValue(":" + p_Time, lists.value(p_Time).value(i));
                m_query->bindValue(":" + p_X, lists.value(p_X).value(i));
                m_query->bindValue(":" + p_Y, lists.value(p_Y).value(i));
            }else{
                qDebug() << "Error: not match table. Only support three tables (car, path, pathitem) now";
                return -1;
            }

            bool result = m_query->exec();
            temp = m_query->lastInsertId();

            if (!flag && !records.contains(lists.value(constrains).value(i).toString())){
                records.insert(lists.value(constrains).value(i).toString(),temp);
            }

            if(m_query->lastError().type() != QSqlError::NoError || !result){
                qDebug() << "Operations occur errors: " << m_query->lastError().text();
                m_db->rollback();
                return -1;
            }
        }
    }

    m_db->commit();
    return m_query->numRowsAffected();
}



/**
 * @brief basic function for executing a batch of data
 *
 */
bool DataBaseCore::Execute_Batch(){
    m_db->transaction();

    bool result = m_query->execBatch();
    if(m_query->lastError().type() != QSqlError::NoError || !result){
        qDebug() << "Operations occur errors, " << m_query->lastError().text();
        m_db->rollback();
        return false;
    }

    m_db->commit();
    return true;
}


/**
 *
 * @brief basic function for executing single data
 *
 */
bool DataBaseCore::Execute(){
    m_db->transaction();

    bool queryRes = m_query->exec();
    if (m_query->lastError().type() != QSqlError::NoError || !queryRes){
        qDebug() << "Operation occurs errors, " << m_query->lastError().text();
        m_db->rollback();
        return false;
    }

    m_db->commit();
    return true;
}


void DataBaseCore::disConnect(){
    qDebug() << "Disconnected From Database! ";
    m_query->clear();
    m_db->close();
}


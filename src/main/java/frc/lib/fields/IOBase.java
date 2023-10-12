package frc.lib.fields;

public abstract class IOBase {
    protected final FieldsTable fields;

    protected IOBase(FieldsTable fieldsTable) {
        fields = fieldsTable;
        fieldsTable.setPeriodicBeforeFields(this::periodicBeforeFields);
    }

    protected IOBase(String name) {
        fields = new FieldsTable(name);
    }

    public FieldsTable getFieldsTable() {
        return fields;
    }

    public void periodicBeforeFields() {
    }
}
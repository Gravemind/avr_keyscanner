
var table;

function refresh() {

    $('input.toggle-vis').each(function(){
        var column = table.column( $(this).attr('data-column') );
        column.visible( this.checked );
    });

    table.rows().every(function(){
        var show_row = false;

        var maxv = -Infinity;
        var minv = Infinity;

        this.nodes().toJQuery().find('td').each(function(){
            var t = $(this).text();
            var v = Math.abs(parseFloat(t));
            if (t == '-')
                v = 0;
            if (t != '' && v != 0.0)
                show_row = true;
            if (isNaN(v))
                return;
            maxv = Math.max(maxv, v);
            minv = Math.min(minv, v);
        });

        this.nodes().toJQuery().find('td').each(function(){
            var t = $(this).text();
            var v = Math.abs(parseFloat(t));
            if (t == '-')
                v = 0;
            if (isNaN(v))
                return;
            if (minv == maxv)
                $(this).css('color', '#909090');
            else if (v == minv)
                $(this).css('color', '#009000');
            else if (v == maxv)
                $(this).css('color', '#900000');
            else
                $(this).css('color', '');
        });

        if (show_row)
            this.nodes().to$().show();
        else
            this.nodes().to$().hide();
    });
}

$(document).ready(function() {
    table = $('#table').DataTable({
        "paging": false,
        "order": [[ 0, 'asc' ]],
        "autoWidth": false,
    });

    $('input.toggle-vis').on('click', refresh);

    $('#vis-none').on('click', function(e) {
        e.preventDefault();
        $('input.toggle-vis').each(function () { this.checked = false; });
        refresh();
    });

    $('#vis-all').on('click', function(e) {
        e.preventDefault();
        $('input.toggle-vis').each(function () { this.checked = true; });
        refresh();
    });

    refresh();

});

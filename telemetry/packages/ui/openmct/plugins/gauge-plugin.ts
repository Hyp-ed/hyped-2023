// @ts-nocheck
export function GaugePlugin() {
  var progress_type = 'view.progress-bar';
  var gauge_type = 'view.gauge';

  function ProgressBarView(domain, openmct) {
    var container = null;

    var subscriptions = null;

    var min = domain['value.min'] || 0;
    if (typeof min === 'string') {
      min = parseFloat(min);
    }
    var max = domain['value.max'] || 1;
    if (typeof max === 'string') {
      max = parseFloat(max);
    }

    function getPercentOf(value) {
      return (value - min) / (max - min);
    }

    function createRow(name) {
      var row = document.createElement('tr');
      row.style.width = '100%';
      var head = document.createElement('td');
      head.innerText = name;
      row.appendChild(head);

      var box = document.createElement('td');
      row.appendChild(box);

      var bar = createProgressBar();
      box.appendChild(bar.content);

      return {
        content: row,
        progress: bar,
        setLabel: function (text) {
          head.innerText = text;
        },
      };
    }

    function createProgressBar() {
      var bar = document.createElement('div');
      bar.style.width = '100%';
      bar.style.height = '30px';
      bar.classList.add('plot-display-area');

      var percentage = document.createElement('div');
      bar.appendChild(percentage);
      percentage.classList.add('c-button');
      percentage.classList.add('c-button--major');
      percentage.style.width = '0%';
      percentage.style.height = '100%';
      percentage.style.padding = '0';

      return {
        content: bar,
        setPercent(float) {
          var width = float;
          if (width > 100) {
            width = 100;
          } else if (width < 0) {
            width = 0;
          }
          percentage.style.width = width + '%';
          percentage.innerText = Math.round(float) + '%';
        },
        setNormalizedPercent(float) {
          if (!float) float = 0;
          this.setPercent(float * 100);
        },
      };
    }

    this.show = function (dom) {
      container = document.createElement('table');
      container.style.width = '100%';

      var composition = domain.composition || [];
      subscriptions = [];
      subscriptions.length = composition.length;
      dom.appendChild(container);

      composition.forEach((id, index) => {
        var row = createRow(id.key);
        container.appendChild(row.content);

        openmct.objects.get(id).then(function (cDomain) {
          var allTelemetry = [];
          if (cDomain.telemetry && cDomain.telemetry.values) {
            allTelemetry = cDomain.telemetry.values.filter(
              (value) => value.format === 'float',
            );
          }
          var first = allTelemetry.length > 0 ? allTelemetry[0] : null;

          subscriptions[index] = openmct.telemetry.subscribe(
            cDomain,
            function (data) {
              if (!first) {
                return;
              }

              row.progress.setNormalizedPercent(
                getPercentOf(data[first.source || first.key]),
              );
            },
          );
        });
      });
    };

    this.destroy = function () {
      // Remove dom
      if (container) {
        container.remove();
      }

      // Unsubscribe
      if (subscriptions != null) {
        for (var i = 0; i < subscriptions.length; i++) {
          if (subscriptions[i]) subscriptions[i]();
        }
        subscriptions = null;
      }
    };
    return this;
  }

  function GaugeView(domain, openmct) {
    var container = null;
    this.container = container;

    var min_value = domain['value.min'] || 0;
    if (typeof min_value === 'string') {
      min_value = parseFloat(min_value);
    }
    var max_value = domain['value.max'] || 1;
    if (typeof max_value === 'string') {
      max_value = parseFloat(max_value);
    }

    function getPercentOf(value) {
      return (value - min_value) / (max_value - min_value);
    }

    function makeGauge(name) {
      var gauge = document.createElement('div');
      gauge.style.display = 'inline-block';
      gauge.style.position = 'relative';

      var canvas = document.createElement('canvas');
      var radius = 80;
      var outerLineThickness = 15;
      var innerLineThickness = 6;
      canvas.width = 2 * radius + 2 * outerLineThickness;
      canvas.height = radius + 2 * outerLineThickness;
      var centre = {
        x: canvas.width / 2,
        y: canvas.height - outerLineThickness,
      };
      var context = canvas.getContext('2d');
      gauge.appendChild(canvas);

      function redraw(percent) {
        // Setup
        context.lineCap = 'round';
        context.font = '24px verdana';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillStyle = 'gray';
        context.clearRect(0, 0, canvas.width, canvas.height);

        // Draw full outline
        context.beginPath();
        context.arc(centre.x, centre.y, radius, Math.PI, 0, false),
          (context.strokeStyle = 'lightgray');
        context.lineWidth = outerLineThickness;
        context.stroke();

        // Draw fill percentage
        if (percent > 0) {
          context.beginPath();
          context.arc(
            centre.x,
            centre.y,
            radius,
            Math.PI,
            -Math.PI * (1 - percent),
            false,
          ),
            (context.strokeStyle = 'blue');
          context.lineWidth = innerLineThickness;
          context.stroke();
        }
      }
      redraw(0);

      // Draw text
      var row = document.createElement('tr');

      var min = document.createElement('td');
      min.innerText = min_value;
      min.style.textAlign = 'left';
      row.appendChild(min);

      var title = document.createElement('td');
      title.innerText = name;
      title.style.textAlign = 'center';
      row.appendChild(title);

      var max = document.createElement('td');
      max.innerText = max_value;
      max.style.textAlign = 'right';
      row.appendChild(max);

      var bottom = document.createElement('table');
      bottom.appendChild(row);
      gauge.appendChild(bottom);

      var valuebox = document.createElement('div');
      valuebox.innerText = 0;
      valuebox.style.paddingTop = radius + 'px';
      valuebox.style.verticalAlign = 'bottom';
      valuebox.style.font = '24px verdana';
      valuebox.style.textAlign = 'center';
      valuebox.style.position = 'absolute';
      valuebox.style.left = 0;
      valuebox.style.right = 0;
      valuebox.style.top = 0;
      valuebox.style.bottom = 0;
      gauge.appendChild(valuebox);

      return {
        content: gauge,
        setLabel: function (text) {
          title.innerText = text;
        },
        setValue(value) {
          valuebox.innerText = Math.round((value + Number.EPSILON) * 100) / 100;

          var percentage = getPercentOf(value);
          if (percentage < 0) {
            percentage = 0;
          } else if (percentage > 1) {
            percentage = 1;
          }

          redraw(percentage);
        },
      };
    }

    var subscriptions = null;

    this.show = function (dom) {
      container = document.createElement('div');
      var composition = domain.composition || [];
      subscriptions = [];
      subscriptions.length = composition.length;
      dom.appendChild(container);

      composition.forEach((id, index) => {
        var gauge = makeGauge(id.key);
        container.appendChild(gauge.content);

        openmct.objects.get(id).then(function (cDomain) {
          var allTelemetry = [];
          if (cDomain.telemetry && cDomain.telemetry.values) {
            allTelemetry = cDomain.telemetry.values.filter(
              (value) => value.format === 'float',
            );
          }
          var first = allTelemetry.length > 0 ? allTelemetry[0] : null;

          subscriptions[index] = openmct.telemetry.subscribe(
            cDomain,
            function (data) {
              if (!first) {
                return;
              }
              var value = data[first.source || first.key];
              if (typeof value === 'number') {
                gauge.setValue(value);
              }
            },
          );
        });
      });
    };

    this.destroy = function () {
      // Remove dom
      if (container) {
        container.remove();
      }

      // Unsubscribe
      if (subscriptions != null) {
        for (var i = 0; i < subscriptions.length; i++) {
          if (subscriptions[i]) subscriptions[i]();
        }
        subscriptions = null;
      }
    };
    return this;
  }

  return function install(openmct) {
    openmct.types.addType(progress_type, {
      name: 'Progress Bar',
      description:
        'Progress bars indicate visually what percentage of a resource is used on a horizontal bar',
      creatable: true,
      initialize: function (domain) {
        domain.composition = [];
      },
      form: [
        {
          key: 'value.min',
          name: 'Min Value',
          control: 'textfield',
          cssClass: 'l-input-lg',
        },
        {
          key: 'value.max',
          name: 'Max Value',
          control: 'textfield',
          cssClass: 'l-input-lg',
        },
      ],
    });
    openmct.objectViews.addProvider({
      key: progress_type,
      name: 'Progress Bar View',
      canView: function (domain) {
        return domain.type === progress_type;
      },
      view: function (domain) {
        return new ProgressBarView(domain, openmct);
      },
      canEdit: function (domain) {
        return false;
      },
      priority: function () {
        return 1;
      },
    });

    openmct.types.addType(gauge_type, {
      name: 'Gauge',
      description:
        'Gauges indicate visually what percentage of a resource is used on a circular dial',
      cssClass: 'icon-timer',
      creatable: true,
      initialize: function (domain) {
        domain.composition = [];
      },
      form: [
        {
          key: 'value.min',
          name: 'Min Value',
          control: 'textfield',
          cssClass: 'l-input-lg',
        },
        {
          key: 'value.max',
          name: 'Max Value',
          control: 'textfield',
          cssClass: 'l-input-lg',
        },
      ],
    });

    openmct.objectViews.addProvider({
      key: gauge_type,
      name: 'Gauge View',
      canView: function (domain) {
        return domain.type === gauge_type;
      },
      view: function (domain) {
        return new GaugeView(domain, openmct);
      },
      canEdit: function (domain) {
        return false;
      },
      priority: function () {
        return 1;
      },
    });
  };
}
